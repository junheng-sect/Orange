"""Serial barcode reader framing, independent of ROS and the serial port."""

TEXT_TERMINATORS = b'\r\n'


def parse_hex(text):
    """Parse '02 F4 03' (spaces optional) into bytes; None/empty gives b''."""
    if text is None:
        return b''
    if not isinstance(text, str):
        raise ValueError('hex value must be a string, got %r' % (text,))
    digits = ''.join(text.split())
    if len(digits) % 2:
        raise ValueError('hex value has an odd number of digits: %r' % text)
    try:
        return bytes.fromhex(digits)
    except ValueError:
        raise ValueError('invalid hex value: %r' % text)


class FrameParser:
    """Split reader output into frames.

    Delimited mode (reply_start/reply_end both set): bytes before the start
    delimiter are noise; the payload is between the delimiters.
    Text mode (both empty): a frame ends at CR or LF; blank lines are skipped.
    """

    def __init__(self, start=b'', end=b'', max_frame=4096):
        if bool(start) != bool(end):
            raise ValueError('reply_start_hex and reply_end_hex must both be set or both empty')
        self.start = bytes(start)
        self.end = bytes(end)
        self.max_frame = max_frame
        self.buffer = bytearray()

    @property
    def text_mode(self):
        return not self.start

    def reset(self):
        self.buffer.clear()

    def feed(self, data):
        """Append data; return (frames, dropped_noise_bytes)."""
        self.buffer += data
        if self.text_mode:
            return self._feed_text()
        return self._feed_delimited()

    def _feed_text(self):
        frames, dropped = [], 0
        while True:
            index = min((i for i in (self.buffer.find(t) for t in TEXT_TERMINATORS) if i >= 0),
                        default=-1)
            if index < 0:
                break
            line = bytes(self.buffer[:index])
            del self.buffer[:index + 1]
            if line:
                frames.append(line)
        if len(self.buffer) > self.max_frame:
            dropped += len(self.buffer)
            self.buffer.clear()
        return frames, dropped

    def _feed_delimited(self):
        frames, dropped = [], 0
        while True:
            begin = self.buffer.find(self.start)
            if begin < 0:
                # Keep a possible partial start delimiter at the tail.
                keep = len(self.start) - 1
                noise = len(self.buffer) - keep if len(self.buffer) > keep else 0
                if noise:
                    dropped += noise
                    del self.buffer[:noise]
                break
            if begin:
                dropped += begin
                del self.buffer[:begin]
            finish = self.buffer.find(self.end, len(self.start))
            if finish < 0:
                if len(self.buffer) > self.max_frame:
                    dropped += len(self.buffer)
                    self.buffer.clear()
                break
            frames.append(bytes(self.buffer[len(self.start):finish]))
            del self.buffer[:finish + len(self.end)]
        return frames, dropped
