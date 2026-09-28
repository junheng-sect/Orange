"""Deliberately contains no sockets, HTTP clients, or hardware PWM writes."""
from collections import deque
from dataclasses import dataclass
from .controller import finite_number


@dataclass(frozen=True)
class MockCommand:
    monotonic_time: float
    mode: str
    duty: float


class MockTransport:
    name = 'mock'
    hardware_feedback_valid = False

    def __init__(self):
        self.commands = deque(maxlen=1000)
        self.last_command = None

    def update(self, status, now):
        if not status.enabled or status.target_duty is None:
            # Invalidating a mock command is NOT a physical stop/zero-duty command.
            self.last_command = None
            return
        duty = finite_number(status.target_duty, 'duty')
        if not 0 <= duty <= 1:
            raise ValueError('invalid duty')
        command = MockCommand(finite_number(now, 'time'), status.mode, duty)
        self.commands.append(command)
        self.last_command = command


def make_transport(name):
    if name != 'mock':
        raise ValueError('unsupported transport %r: only mock is implemented; '
                         'ESP32 protocol and fail-safe output are not defined' % name)
    return MockTransport()
