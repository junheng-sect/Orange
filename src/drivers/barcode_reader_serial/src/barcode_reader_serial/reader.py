"""Trigger a serial barcode reader, wait for one frame, then release it.

Behaviour follows DroneVision BarcodeReaderSerial: one request at a time,
stale/unsolicited bytes are dropped, and the release command is sent after
success, timeout, empty frame, no-read reply and cancel. The port object is
injected so the logic can be tested without hardware.
"""
import collections
import logging
import math
import threading
import time

from .protocol import FrameParser, parse_hex

SERIAL_UNAVAILABLE = 'serial_unavailable'
BUSY = 'busy'
NOT_CONFIGURED = 'not_configured'
TIMEOUT = 'timeout'
EMPTY_FRAME = 'empty_frame'
NO_READ = 'no_read'
CANCELLED = 'cancelled'
WRITE_FAILED = 'write_failed'
IO_ERROR = 'io_error'

PARITY = {'none': 'N', 'even': 'E', 'odd': 'O', 'mark': 'M', 'space': 'S'}
STOP_BITS = {'one': 1, 'onepointfive': 1.5, 'two': 2}
PAYLOAD_STRIP = b' \t\r\n\x00'

ReaderConfig = collections.namedtuple('ReaderConfig', [
    'port', 'baud', 'data_bits', 'parity', 'stop_bits', 'trigger', 'release',
    'reply_start', 'reply_end', 'read_timeout_s', 'no_read_reply'])

ReadResult = collections.namedtuple('ReadResult', [
    'success', 'code', 'error', 'elapsed_s', 'request_seq', 'raw'])


def config_from_params(params):
    """Build ReaderConfig from DroneVision-style serial_reader keys."""
    known = {'port', 'baud', 'data_bits', 'parity', 'stop_bits', 'trigger_hex',
             'trigger_command', 'release_hex', 'reply_start_hex', 'reply_end_hex',
             'read_timeout_ms', 'no_read_reply'}
    unknown = set(params) - known
    if unknown:
        raise ValueError('unknown reader parameters: %s' % ', '.join(sorted(unknown)))
    port = params.get('port', '/dev/ttyS1')
    if not isinstance(port, str) or not port:
        raise ValueError('port must be a non-empty string')
    baud = params.get('baud', 115200)
    if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
        raise ValueError('baud must be a positive integer')
    data_bits = params.get('data_bits', 8)
    if data_bits not in (5, 6, 7, 8) or isinstance(data_bits, bool):
        raise ValueError('data_bits must be 5, 6, 7 or 8')
    parity = PARITY.get(str(params.get('parity', 'None')).lower())
    if parity is None:
        raise ValueError('parity must be one of None/Even/Odd/Mark/Space')
    stop_bits = STOP_BITS.get(str(params.get('stop_bits', 'One')).lower())
    if stop_bits is None:
        raise ValueError('stop_bits must be one of One/OnePointFive/Two')
    trigger = parse_hex(params.get('trigger_hex', '02 F4 03'))
    if not trigger:
        command = params.get('trigger_command', '')
        if not isinstance(command, str):
            raise ValueError('trigger_command must be a string')
        trigger = command.encode('utf-8')
    timeout_ms = params.get('read_timeout_ms', 3500)
    if isinstance(timeout_ms, bool) or not isinstance(timeout_ms, (int, float)) \
            or not math.isfinite(timeout_ms) or not 0 < timeout_ms <= 30000:
        raise ValueError('read_timeout_ms must be in (0, 30000]')
    reply_start = parse_hex(params.get('reply_start_hex', ''))
    reply_end = parse_hex(params.get('reply_end_hex', ''))
    FrameParser(reply_start, reply_end)  # validates the delimiter pair
    no_read_reply = params.get('no_read_reply', 'NR')
    if not isinstance(no_read_reply, str):
        raise ValueError('no_read_reply must be a string')
    return ReaderConfig(port, baud, data_bits, parity, stop_bits, trigger,
                        parse_hex(params.get('release_hex', '02 F5 03')),
                        reply_start, reply_end, timeout_ms / 1000.0,
                        no_read_reply.encode('utf-8').strip(PAYLOAD_STRIP))


def open_serial(config):
    import serial
    return serial.Serial(config.port, baudrate=config.baud, bytesize=config.data_bits,
                         parity=config.parity, stopbits=config.stop_bits,
                         timeout=0.05, write_timeout=0.5)


class _Request:
    def __init__(self, seq):
        self.seq = seq
        self.event = threading.Event()
        self.result = None  # (error, raw)


class BarcodeReader:
    def __init__(self, config, port_factory=open_serial, log=None):
        self.config = config
        self.port_factory = port_factory
        self.log = log or logging.getLogger(__name__)
        self.parser = FrameParser(config.reply_start, config.reply_end)
        self.lock = threading.Lock()
        self.port = None
        self.pending = None
        self.seq = 0
        self.closed = False
        self.thread = threading.Thread(target=self._read_loop, name='barcode-reader-rx',
                                       daemon=True)

    def start(self):
        with self.lock:
            if not self._open_locked():
                self.log.warning('Barcode reader unavailable; node keeps running and '
                                 'retries opening %s on the next read', self.config.port)
        self.thread.start()

    def read(self, timeout_s=None):
        """Trigger once and block until a frame, timeout or cancel."""
        started = time.monotonic()
        timeout_s = self.config.read_timeout_s if timeout_s is None else timeout_s
        with self.lock:
            if self.closed:
                return self._result(started, 0, SERIAL_UNAVAILABLE)
            if self.pending is not None:
                return self._result(started, 0, BUSY)
            if not self.config.trigger:
                return self._result(started, 0, NOT_CONFIGURED)
            if self.port is None and not self._open_locked():
                return self._result(started, 0, SERIAL_UNAVAILABLE)
            self.seq += 1
            request = _Request(self.seq)
            self.pending = request
            # Drop residual bytes from earlier requests before triggering.
            self.parser.reset()
            try:
                self.port.reset_input_buffer()
                self.port.write(self.config.trigger)
                self.port.flush()
            except Exception as error:  # pyserial raises SerialException/OSError
                self.pending = None
                self._drop_port_locked('trigger write failed: %s' % error)
                return self._result(started, request.seq, WRITE_FAILED)
        self.log.info('Barcode reader trigger sent request=%d', request.seq)
        request.event.wait(timeout_s)
        with self.lock:
            if request.result is None:
                request.result = (TIMEOUT, b'')
            if self.pending is request:
                self.pending = None
            error, raw = request.result
            if error != IO_ERROR:
                self._release_locked()
        result = self._result(started, request.seq, error, raw)
        if result.success:
            self.log.info('Barcode read request=%d: %s', request.seq, result.code)
        elif error == NO_READ:
            # 读码器正常回复“读不到码”，是测距/对准实验里的正常结果，不按异常告警。
            self.log.info('Barcode read failed request=%d: %s', request.seq, error)
        else:
            self.log.warning('Barcode read failed request=%d: %s', request.seq, error)
        return result

    def cancel(self):
        with self.lock:
            if self.pending is None:
                return False
            self._complete_locked(self.pending, CANCELLED)
            return True

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            if self.pending is not None:
                self._complete_locked(self.pending, CANCELLED)
                self.pending = None
                self._release_locked()
            if self.port is not None:
                self._close_port(self.port)
                self.port = None
        if self.thread.is_alive():
            self.thread.join(1.0)

    def _read_loop(self):
        while True:
            with self.lock:
                if self.closed:
                    return
                port = self.port
            if port is None:
                time.sleep(0.05)
                continue
            try:
                data = port.read(max(1, port.in_waiting))
            except Exception as error:
                with self.lock:
                    if self.port is port:
                        self._drop_port_locked('receive failed: %s' % error)
                continue
            if data:
                with self.lock:
                    if self.port is port:
                        self._on_data_locked(data)

    def _on_data_locked(self, data):
        if self.pending is None:
            self.parser.reset()
            self.log.debug('Ignoring %d bytes outside a read request', len(data))
            return
        frames, dropped = self.parser.feed(data)
        if dropped:
            self.log.debug('Dropped %d noise bytes before frame', dropped)
        if frames:
            payload = frames[0].strip(PAYLOAD_STRIP)
            if not payload:
                error = EMPTY_FRAME
            elif payload == self.config.no_read_reply:
                error = NO_READ  # the reader's own "nothing decoded" answer, e.g. NR
            else:
                error = ''
            self._complete_locked(self.pending, error, frames[0])

    def _complete_locked(self, request, error, raw=b''):
        if request.result is None:
            request.result = (error, raw)
            request.event.set()

    def _open_locked(self):
        try:
            self.port = self.port_factory(self.config)
        except Exception as error:
            self.port = None
            self.log.warning('Failed to open barcode reader %s: %s', self.config.port, error)
            return False
        self.log.info('Opened barcode reader %s @ %d bps', self.config.port, self.config.baud)
        return True

    def _drop_port_locked(self, reason):
        self.log.warning('Barcode reader serial error, closing port: %s', reason)
        if self.pending is not None:
            self._complete_locked(self.pending, IO_ERROR)
        if self.port is not None:
            self._close_port(self.port)
            self.port = None
        self.parser.reset()

    def _release_locked(self):
        if not self.config.release or self.port is None:
            return
        try:
            self.port.write(self.config.release)
            self.port.flush()
        except Exception as error:
            self._drop_port_locked('release write failed: %s' % error)

    @staticmethod
    def _close_port(port):
        try:
            port.close()
        except Exception:
            pass

    @staticmethod
    def _result(started, seq, error, raw=b''):
        code = raw.strip(PAYLOAD_STRIP).decode('utf-8', 'replace') if not error else ''
        return ReadResult(not error, code, error, time.monotonic() - started, seq, raw)
