#!/usr/bin/env python3
import threading
import time
import unittest

from barcode_reader_serial import reader as rd
from barcode_reader_serial.reader import BarcodeReader, config_from_params

TRIGGER = b'\x02\xf4\x03'
RELEASE = b'\x02\xf5\x03'


class FakePort:
    """In-memory serial port; `reply` is sent back after each trigger."""

    def __init__(self, reply=None, delay=0.05):
        self.cond = threading.Condition()
        self.rx = bytearray()
        self.writes = []
        self.closed = False
        self.reply = reply
        self.delay = delay
        self.fail_read = False

    @property
    def in_waiting(self):
        with self.cond:
            return len(self.rx)

    def read(self, size=1):
        with self.cond:
            self.cond.wait_for(lambda: self.rx or self.closed or self.fail_read, 0.05)
            if self.closed or self.fail_read:
                raise OSError('device disconnected')
            data = bytes(self.rx[:size])
            del self.rx[:size]
            return data

    def write(self, data):
        self.writes.append(bytes(data))
        if data == TRIGGER and self.reply is not None:
            threading.Timer(self.delay, self.inject, (self.reply,)).start()
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        with self.cond:
            self.rx.clear()

    def inject(self, data):
        with self.cond:
            self.rx += data
            self.cond.notify_all()

    def close(self):
        with self.cond:
            self.closed = True
            self.cond.notify_all()


def make_reader(port=None, factory=None, **params):
    params.setdefault('read_timeout_ms', 300)
    reader = BarcodeReader(config_from_params(params),
                           port_factory=factory or (lambda _config: port))
    reader.start()
    return reader


class Config(unittest.TestCase):
    def test_defaults_match_dronevision(self):
        config = config_from_params({})
        self.assertEqual((config.port, config.baud, config.trigger, config.release),
                         ('/dev/ttyS1', 115200, TRIGGER, RELEASE))
        # read_timeout raised from Drone-Vision's 1500 ms: the reader itself answers NR after ~3 s.
        self.assertEqual((config.reply_start, config.reply_end, config.read_timeout_s),
                         (b'', b'', 3.5))
        self.assertEqual(config.no_read_reply, b'NR')

    def test_text_trigger(self):
        config = config_from_params({'trigger_hex': '', 'trigger_command': 'T\r'})
        self.assertEqual(config.trigger, b'T\r')

    def test_invalid(self):
        for params in ({'baud': 0}, {'parity': 'x'}, {'stop_bits': 3},
                       {'read_timeout_ms': 0}, {'reply_start_hex': '02'}, {'bogus': 1},
                       {'no_read_reply': 1}):
            with self.assertRaises(ValueError, msg=params):
                config_from_params(params)


class Reader(unittest.TestCase):
    def tearDown(self):
        if getattr(self, 'reader', None):
            self.reader.close()

    def test_success_sends_trigger_then_release(self):
        port = FakePort(b'NEXT9017324\r')
        self.reader = make_reader(port)
        result = self.reader.read()
        self.assertTrue(result.success, result)
        self.assertEqual((result.code, result.error, result.request_seq), ('NEXT9017324', '', 1))
        self.assertEqual(result.raw, b'NEXT9017324')
        self.assertEqual(port.writes, [TRIGGER, RELEASE])

    def test_timeout_releases(self):
        port = FakePort()
        self.reader = make_reader(port)
        result = self.reader.read(0.2)
        self.assertEqual(result.error, rd.TIMEOUT)
        self.assertGreaterEqual(result.elapsed_s, 0.2)
        self.assertEqual(port.writes, [TRIGGER, RELEASE])

    def test_busy_and_cancel(self):
        port = FakePort()
        self.reader = make_reader(port)
        results = []
        worker = threading.Thread(target=lambda: results.append(self.reader.read(5.0)))
        worker.start()
        deadline = time.monotonic() + 1
        while self.reader.pending is None and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(self.reader.read().error, rd.BUSY)
        self.assertTrue(self.reader.cancel())
        worker.join(1)
        self.assertEqual(results[0].error, rd.CANCELLED)
        self.assertLess(results[0].elapsed_s, 1.0)
        self.assertEqual(port.writes, [TRIGGER, RELEASE])
        self.assertFalse(self.reader.cancel())

    def test_late_reply_is_not_used_by_next_request(self):
        port = FakePort()
        self.reader = make_reader(port)
        self.assertEqual(self.reader.read(0.1).error, rd.TIMEOUT)
        port.inject(b'LATE\r')  # arrives after the timeout, outside any request
        time.sleep(0.15)
        port.inject(b'PART')  # unterminated leftovers still in the port buffer
        port.reply = b'FRESH\r'
        result = self.reader.read()
        self.assertEqual(result.code, 'FRESH')
        self.assertEqual(result.request_seq, 2)

    def test_delimited_mode_with_noise(self):
        port = FakePort(b'\xff\x00\x02CODE-1\x03')
        self.reader = make_reader(port, reply_start_hex='02', reply_end_hex='03')
        self.assertEqual(self.reader.read().code, 'CODE-1')

    def test_empty_frame(self):
        for params, reply in (({}, b'  \r'),
                              ({'reply_start_hex': '02', 'reply_end_hex': '03'}, b'\x02\x03')):
            port = FakePort(reply)
            self.reader = make_reader(port, **params)
            result = self.reader.read()
            self.assertEqual(result.error, rd.EMPTY_FRAME)
            self.assertEqual(port.writes, [TRIGGER, RELEASE])
            self.reader.close()

    def test_no_read_reply_is_failure(self):
        for params, reply, raw in (({}, b'NR\r', b'NR'),
                                   ({'no_read_reply': 'NOREAD'}, b' NOREAD \r\n', b' NOREAD '),
                                   ({'reply_start_hex': '02', 'reply_end_hex': '03'},
                                    b'\x02NR\x03', b'NR')):
            port = FakePort(reply)
            self.reader = make_reader(port, **params)
            result = self.reader.read()
            self.assertFalse(result.success, params)
            self.assertEqual((result.code, result.error), ('', rd.NO_READ), params)
            self.assertEqual(result.raw, raw)
            self.assertEqual(port.writes, [TRIGGER, RELEASE])
            self.reader.close()

    def test_no_read_only_matches_whole_payload(self):
        for params, reply, code in (({}, b'NR123\r', 'NR123'),
                                    ({}, b'nr\r', 'nr'),
                                    ({'no_read_reply': ''}, b'NR\r', 'NR')):
            port = FakePort(reply)
            self.reader = make_reader(port, **params)
            result = self.reader.read()
            self.assertTrue(result.success, (params, result))
            self.assertEqual(result.code, code)
            self.reader.close()

    def test_not_configured(self):
        port = FakePort()
        self.reader = make_reader(port, trigger_hex='', trigger_command='')
        self.assertEqual(self.reader.read().error, rd.NOT_CONFIGURED)
        self.assertEqual(port.writes, [])

    def test_open_failure_then_recovery(self):
        port = FakePort(b'OK\r')
        attempts = []

        def factory(_config):
            attempts.append(1)
            if len(attempts) < 3:
                raise OSError('Permission denied')
            return port

        self.reader = make_reader(factory=factory)  # attempt 1 fails at start
        self.assertEqual(self.reader.read().error, rd.SERIAL_UNAVAILABLE)  # attempt 2
        self.assertEqual(self.reader.read().code, 'OK')  # attempt 3 opens
        self.assertEqual(len(attempts), 3)

    def test_io_error_drops_port_and_reopens(self):
        first, second = FakePort(), FakePort(b'AGAIN\r')
        ports = [first, second]
        self.reader = make_reader(factory=lambda _config: ports.pop(0))
        threading.Timer(0.1, lambda: setattr(first, 'fail_read', True)).start()
        result = self.reader.read(2.0)
        self.assertEqual(result.error, rd.IO_ERROR)
        self.assertLess(result.elapsed_s, 1.0)
        self.assertEqual(first.writes, [TRIGGER])
        self.assertEqual(self.reader.read().code, 'AGAIN')

    def test_close_cancels_pending_and_releases(self):
        port = FakePort()
        self.reader = make_reader(port)
        results = []
        worker = threading.Thread(target=lambda: results.append(self.reader.read(5.0)))
        worker.start()
        time.sleep(0.1)
        self.reader.close()
        worker.join(1)
        self.assertEqual(results[0].error, rd.CANCELLED)
        self.assertEqual(port.writes, [TRIGGER, RELEASE])
        self.assertTrue(port.closed)
        self.assertEqual(self.reader.read().error, rd.SERIAL_UNAVAILABLE)

    def test_utf8_replacement(self):
        port = FakePort(b'\xe4\xb8\xad\xff\r')
        self.reader = make_reader(port)
        self.assertEqual(self.reader.read().code, '中�')


if __name__ == '__main__':
    unittest.main()
