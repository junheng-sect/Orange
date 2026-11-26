#!/usr/bin/env python3
import socket
import threading
import time
import unittest
from unittest.mock import patch

from tether_winch_control.controller import Status
from tether_winch_control.transport import TcpTransport


def state(duty=None):
    return Status('HIGH' if duty is not None else 'FAULT', duty is not None,
                  True, '', 2.0, -0.1, duty)


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError('condition timed out')


class Server:
    def __init__(self, port=0):
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('127.0.0.1', port))
        self.port = self.socket.getsockname()[1]
        self.socket.listen(10)
        self.socket.settimeout(0.05)
        self.messages = []
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            try:
                connection, _ = self.socket.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with connection:
                connection.settimeout(1)
                payload = b''
                try:
                    while True:
                        chunk = connection.recv(1024)
                        if not chunk:
                            break
                        payload += chunk
                    self.messages.append(payload)
                except OSError:
                    pass

    def close(self):
        self.stop.set()
        self.socket.close()
        self.thread.join(2)


class TransportTests(unittest.TestCase):
    def test_wire_resend_disable_and_shutdown(self):
        server = Server()
        transport = TcpTransport(host='127.0.0.1', port=server.port, resend_sec=0.08)
        try:
            transport.start()
            wait_for(lambda: len(server.messages) >= 2)
            self.assertEqual(server.messages[:2], [b'DUTY=100#'] * 2)
            self.assertTrue(transport.prepare_enable())
            transport.update(state(300), time.monotonic())
            wait_for(lambda: b'DUTY=300#' in server.messages)
            transport.update(state(), time.monotonic())
            wait_for(lambda: server.messages[-1] == b'DUTY=100#')
            transport.update(state(500), time.monotonic())
            wait_for(lambda: b'DUTY=500#' in server.messages)
            transport.update(state(0), time.monotonic())
            wait_for(lambda: b'DUTY=0#' in server.messages)
            transport.close()
            wait_for(lambda: server.messages[-1] == b'DUTY=100#')
            self.assertFalse(transport.thread.is_alive())
            self.assertEqual(transport.snapshot()['last_sent_duty'], 100)
            self.assertGreater(transport.snapshot()['last_send_time'], 0)
            self.assertFalse(transport.hardware_feedback_valid)
            self.assertTrue(all(m in (b'DUTY=0#', b'DUTY=100#', b'DUTY=300#', b'DUTY=500#')
                                for m in server.messages))
        finally:
            transport.close()
            server.close()

    def test_network_failure_latches_and_recovery_needs_enable(self):
        server = Server()
        port = server.port
        server.close()
        errors = []
        transport = TcpTransport(host='127.0.0.1', port=port, resend_sec=0.05,
                                 on_error=errors.append)
        try:
            transport.start()
            wait_for(lambda: bool(errors))
            self.assertFalse(transport.prepare_enable())
            transport.update(state(300), time.monotonic())
            server = Server(port)
            wait_for(lambda: len(server.messages) >= 2)
            self.assertEqual(set(server.messages), {b'DUTY=100#'})
            self.assertEqual(len(errors), 1)
            self.assertTrue(transport.fault_latched)
            self.assertTrue(transport.prepare_enable())
            transport.update(state(300), time.monotonic())
            wait_for(lambda: b'DUTY=300#' in server.messages)
        finally:
            transport.close()
            server.close()

    def test_replacement_during_connect_drops_old_high(self):
        entered, release = threading.Event(), threading.Event()
        writes = []

        class Connection:
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def settimeout(self, timeout): pass
            def connect(self, address):
                entered.set()
                release.wait(2)
            def sendall(self, data): writes.append(data)
            def shutdown(self, how): pass

        transport = TcpTransport(host='127.0.0.1', resend_sec=0.05)
        transport.update(state(300), 1)
        with patch('tether_winch_control.transport.socket.socket', return_value=Connection()):
            try:
                transport.start()
                self.assertTrue(entered.wait(1))
                before = time.monotonic()
                transport.update(state(), 2)
                self.assertLess(time.monotonic() - before, 0.1)
                release.set()
                wait_for(lambda: bool(writes))
                self.assertEqual(set(writes), {b'DUTY=100#'})
            finally:
                release.set()
                transport.close()

    def test_configuration(self):
        for kwargs in ({'port': True}, {'port': 0}, {'timeout_sec': 0},
                       {'timeout_sec': float('nan')}, {'resend_sec': 0},
                       {'host': 'hostname'}, {'fallback_duty': 501}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                TcpTransport(**kwargs)


if __name__ == '__main__':
    unittest.main()
