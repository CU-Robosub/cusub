import unittest
from wldvl.protocol import WlProtocolParser, WlProtocolChecksumError, WlDVLBase


class TestProtocolLowLevel(unittest.TestCase):

    def test_parse_invalid_sentence(self):

        parser = WlProtocolParser()
        sentence = parser.parse(b"wrbogous\n")
        self.assertEqual(sentence, None)

    def test_parse_velocity_valid(self):
        expected = {"time": 125.0, "vx": 0.05, "vy": 0.01, "vz": 0.001, "fom": 0.5, "altitude": 0.1, "valid": True}

        parser = WlProtocolParser()
        sentence = parser.parse(b"wrx,125,0.05,0.01,0.001,0.5,0.1,y")
        self.assertEqual(expected, sentence)

    def test_parse_with_invalid_checksum(self):
        parser = WlProtocolParser()
        buf = b"wrx,125,0.05,0.01,0.001,0.5,0.1,y*12"
        self.assertRaises(WlProtocolChecksumError, parser.parse, buf)

    def test_parse_with_correct_checksum(self):
        expected = {"time": 125.0, "vx": 0.05, "vy": 0.01, "vz": 0.001, "fom": 0.5, "altitude": 0.1, "valid": True}

        parser = WlProtocolParser()
        buf = b"wrx,125,0.05,0.01,0.001,0.5,0.1,y*6e"
        sentence = parser.parse(buf)
        self.assertEqual(expected, sentence)



class MockIODev():
    """ Mock io device for simulator and unit testing """
    def __init__(self, in_buf):
        self.in_buf = bytearray(in_buf)
        self.out_buf = bytearray()

    @property
    def in_waiting(self):
        return len(self.in_buf)

    def readline(self):
        if self.in_buf:
            buf = bytearray()
            while len(self.in_buf) > 0:
                data = self.in_buf.pop(0)
                if data == ord("\n"):
                    break
                buf.append(data)

            return bytes(buf)

    def feed(self, data):
        for x in bytes(data):
            self.in_buf.append(x)

    @property
    def port(self):
        return "MockPort"


class TestProtocol(unittest.TestCase):
    def test_protocol(self):
        iodev = MockIODev(b"wrx,125,0.05,0.01,0.001,0.5,0.1,y\n")
        dut = WlDVLBase(iodev)

        result = dut.read()
        expected = {"time": 125.0, "vx": 0.05, "vy": 0.01, "vz": 0.001, "fom": 0.5, "altitude": 0.1, "valid": True}

        self.assertEqual(expected, result)
