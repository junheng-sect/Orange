"""Latest-target TCP sender. Socket writes are not hardware acknowledgements."""
from collections import deque
from dataclasses import dataclass
import math
import socket
import threading
import time

from .controller import finite_number, validate_duty


@dataclass(frozen=True)
class MockCommand:
    monotonic_time: float
    mode: str
    duty: int


class MockTransport:
    name = 'mock'
    hardware_feedback_valid = False

    def __init__(self, fallback_duty=100, **unused):
        self.fallback_duty = validate_duty(fallback_duty)
        self.commands = deque(maxlen=1000)
        self.last_command = None

    def start(self):
        pass

    def update(self, status, now):
        duty = status.target_duty if status.enabled else self.fallback_duty
        command = MockCommand(finite_number(now, 'time'), status.mode, validate_duty(duty))
        self.commands.append(command)
        self.last_command = command

    def snapshot(self):
        return dict(last_sent_duty=math.nan, last_send_time=0.0, communication_error='')

    def prepare_enable(self):
        return True

    def close(self):
        self.last_command = MockCommand(time.monotonic(), 'DISABLED', self.fallback_duty)


class TcpTransport:
    name = 'tcp'
    hardware_feedback_valid = False

    def __init__(self, fallback_duty=100, host='192.168.31.87', port=8080,
                 timeout_sec=0.2, resend_sec=1.0, on_error=None):
        self.fallback_duty = validate_duty(fallback_duty, 'fallback_duty')
        # Numeric IPv4 avoids an unbounded DNS lookup in the sender thread.
        try:
            socket.inet_pton(socket.AF_INET, host)
        except (OSError, TypeError):
            raise ValueError('tcp.host must be a numeric IPv4 address')
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError('tcp.port must be an integer in [1, 65535]')
        self.timeout = finite_number(timeout_sec, 'tcp.timeout_sec')
        self.resend = finite_number(resend_sec, 'tcp.resend_sec')
        if not 0 < self.timeout <= 5 or self.resend < 0.05:
            raise ValueError('require 0 < timeout_sec <= 5 and resend_sec >= 0.05')
        self.address = (host, port)
        self.on_error = on_error
        self.cv = threading.Condition()
        self.target = self.fallback_duty
        self.generation = 0
        self.stopping = False
        self.started = False
        self.fault_latched = False
        self.fault_pending = False
        self.last_sent_duty = math.nan
        self.last_send_time = 0.0
        self.communication_error = ''
        self.thread = threading.Thread(target=self._run, name='winch-tcp', daemon=True)

    def start(self):
        with self.cv:
            if not self.started and not self.stopping:
                self.started = True
                self.thread.start()

    def update(self, status, now):
        duty = validate_duty(status.target_duty if status.enabled else self.fallback_duty)
        with self.cv:
            if self.stopping:
                return
            if self.fault_latched:
                duty = self.fallback_duty
            if duty != self.target:
                self.target = duty
                self.generation += 1
                self.cv.notify_all()

    def prepare_enable(self):
        with self.cv:
            if (self.stopping or self.fault_pending or self.communication_error
                    or not self.last_send_time):
                return False
            self.fault_latched = False
            return True

    def snapshot(self):
        with self.cv:
            return dict(last_sent_duty=self.last_sent_duty,
                        last_send_time=self.last_send_time,
                        communication_error=self.communication_error)

    def _send(self, duty, generation=None):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as connection:
            connection.settimeout(self.timeout)
            connection.connect(self.address)
            if generation is not None:
                with self.cv:
                    # A replacement during connect must not send the old command.
                    if self.stopping or generation != self.generation:
                        return False
            connection.sendall(('DUTY=%d#' % duty).encode('ascii'))
            connection.shutdown(socket.SHUT_WR)
        with self.cv:
            self.last_sent_duty = duty
            self.last_send_time = time.time()  # Unix wall time, not ROS /clock.
            self.communication_error = ''
        return True

    def _failure(self, error, notify=True):
        with self.cv:
            first = not self.fault_latched
            self.fault_latched = True
            self.fault_pending = first and notify
            self.communication_error = str(error)
            if self.target != self.fallback_duty:
                self.target = self.fallback_duty
                self.generation += 1
        # Never acquire the node lock while holding the transport condition.
        try:
            if first and notify and self.on_error:
                self.on_error('TCP send failed: %s' % error)
        finally:
            with self.cv:
                self.fault_pending = False

    def _run(self):
        sent_generation = -1
        deadline = 0.0
        while True:
            with self.cv:
                while not self.stopping:
                    remaining = deadline - time.monotonic()
                    if self.generation != sent_generation or remaining <= 0:
                        break
                    self.cv.wait(remaining)
                if self.stopping:
                    break
                duty, generation = self.target, self.generation
            try:
                self._send(duty, generation)
            except OSError as error:
                self._failure(error)
            sent_generation = generation
            deadline = time.monotonic() + self.resend
        # Serialized after any in-flight write: shutdown's final command is low.
        try:
            self._send(self.fallback_duty)
        except OSError as error:
            self._failure(error, notify=False)

    def close(self):
        with self.cv:
            if not self.started:
                return
            self.stopping = True
            self.target = self.fallback_duty
            self.generation += 1
            self.cv.notify_all()
        if threading.current_thread() is not self.thread:
            self.thread.join(timeout=4 * self.timeout + 1.0)


def make_transport(name, fallback_duty=100, **options):
    if name == 'mock':
        return MockTransport(fallback_duty)
    if name == 'tcp':
        return TcpTransport(fallback_duty=fallback_duty, **options)
    raise ValueError('unsupported transport %r; choose mock or tcp' % name)
