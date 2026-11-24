#!/usr/bin/env python3
import unittest

from barcode_reader_serial.protocol import FrameParser, parse_hex


class ParseHex(unittest.TestCase):
    def test_spaced_and_compact(self):
        self.assertEqual(parse_hex('02 F4 03'), b'\x02\xf4\x03')
        self.assertEqual(parse_hex('02f403'), b'\x02\xf4\x03')

    def test_empty(self):
        self.assertEqual(parse_hex(''), b'')
        self.assertEqual(parse_hex(None), b'')

    def test_invalid(self):
        for value in ('02 F', 'zz', 2):
            with self.assertRaises(ValueError):
                parse_hex(value)


class TextMode(unittest.TestCase):
    def test_cr_terminated(self):
        parser = FrameParser()
        self.assertEqual(parser.feed(b'NEXT9017324\r'), ([b'NEXT9017324'], 0))

    def test_fragments_and_crlf(self):
        parser = FrameParser()
        self.assertEqual(parser.feed(b'NEXT90'), ([], 0))
        self.assertEqual(parser.feed(b'17324\r\nB2\n'), ([b'NEXT9017324', b'B2'], 0))

    def test_blank_lines_skipped(self):
        parser = FrameParser()
        self.assertEqual(parser.feed(b'\r\n\r\nX\r'), ([b'X'], 0))

    def test_whitespace_line_is_a_frame(self):
        self.assertEqual(FrameParser().feed(b'  \r'), ([b'  '], 0))

    def test_overflow_dropped(self):
        parser = FrameParser(max_frame=8)
        self.assertEqual(parser.feed(b'0123456789'), ([], 10))
        self.assertEqual(parser.feed(b'OK\r'), ([b'OK'], 0))

    def test_reset(self):
        parser = FrameParser()
        parser.feed(b'OLD')
        parser.reset()
        self.assertEqual(parser.feed(b'NEW\r'), ([b'NEW'], 0))


class DelimitedMode(unittest.TestCase):
    def test_noise_before_start(self):
        parser = FrameParser(b'\x02', b'\x03')
        self.assertEqual(parser.feed(b'\xffab\x02CODE\x03'), ([b'CODE'], 3))

    def test_fragments(self):
        parser = FrameParser(b'\x02', b'\x03')
        self.assertEqual(parser.feed(b'\x02CO'), ([], 0))
        self.assertEqual(parser.feed(b'DE\x03\x02'), ([b'CODE'], 0))
        self.assertEqual(parser.feed(b'X\x03'), ([b'X'], 0))

    def test_empty_frame(self):
        self.assertEqual(FrameParser(b'\x02', b'\x03').feed(b'\x02\x03'), ([b''], 0))

    def test_multibyte_delimiters_split(self):
        parser = FrameParser(b'\xaa\x55', b'\x0d\x0a')
        self.assertEqual(parser.feed(b'zz\xaa'), ([], 2))
        self.assertEqual(parser.feed(b'\x55AB\x0d'), ([], 0))
        self.assertEqual(parser.feed(b'\x0a'), ([b'AB'], 0))

    def test_unbalanced_delimiters_rejected(self):
        with self.assertRaises(ValueError):
            FrameParser(b'\x02', b'')


if __name__ == '__main__':
    unittest.main()
