"""Pseudo-terminal that behaves like the serial barcode reader."""
import os
import select
import threading
import tty

TRIGGER = b'\x02\xf4\x03'
RELEASE = b'\x02\xf5\x03'


class PtyReaderDevice:
    """Answer each trigger with `reply` (None = stay silent); record commands."""

    def __init__(self, reply=None, link=None):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.link = link
        if link:
            os.symlink(self.path, link)
        self.reply = reply
        self.received = bytearray()
        self.triggers = 0
        self.releases = 0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            ready, _, _ = select.select([self.master], [], [], 0.05)
            if not ready:
                continue
            try:
                data = os.read(self.master, 256)
            except OSError:
                return
            with self.lock:
                self.received += data
                while True:
                    t, r = self.received.find(TRIGGER), self.received.find(RELEASE)
                    hits = [i for i in (t, r) if i >= 0]
                    if not hits:
                        break
                    first = min(hits)
                    del self.received[:first + 3]
                    if first == t:
                        self.triggers += 1
                        if self.reply is not None:
                            os.write(self.master, self.reply)
                    else:
                        self.releases += 1

    def counts(self):
        with self.lock:
            return self.triggers, self.releases

    def close(self):
        self.stop_event.set()
        self.thread.join(1)
        if self.link and os.path.islink(self.link):
            os.unlink(self.link)
        os.close(self.master)
        os.close(self.slave)
