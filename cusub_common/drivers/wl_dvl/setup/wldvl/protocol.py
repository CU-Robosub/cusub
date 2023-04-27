# encoding=utf-8
"""
Water Linked DVL protocol parser
"""
from __future__ import print_function, division
import logging
import time
import sys
import serial
import crcmod
import struct


# Logger
log = logging.getLogger(__file__)

# Python2 detection
IS_PY2 = False
if sys.version_info < (3, 0):
    IS_PY2 = True

# Protocol definitions
SOP = ord('w')
EOP = ord('\n')
DIR_CMD = ord('c')
DIR_RESP = ord('r')
CHECKSUM = ord('*')

CMD_GET_VERSION = ord('v')
CMD_GET_PAYLOAD_SIZE = ord('n')
CMD_GET_BUFFER_LENGTH = ord('l')
CMD_GET_DIAGNOSTIC = ord('d')
CMD_GET_SETTINGS = ord('c')
CMD_SET_SETTINGS = ord('s')
CMD_QUEUE_PACKET = ord('q')
CMD_FLUSH = ord('f')
RESP_GOT_PACKET = ord('p')
VELOCITY_REPORT = ord('x')
ALL_VALID = [
    CMD_GET_VERSION,
    CMD_GET_PAYLOAD_SIZE,
    CMD_GET_BUFFER_LENGTH,
    CMD_GET_DIAGNOSTIC,
    CMD_GET_SETTINGS,
    CMD_SET_SETTINGS,
    CMD_QUEUE_PACKET,
    CMD_FLUSH,
    RESP_GOT_PACKET,
    VELOCITY_REPORT,
]




def is_checksum(ch):
    """ Is the given byte an checksum char """
    #print(type(ch), ch, chr(ch))
    #print("Checksum ", type(CHECKSUM), ord(CHECKSUM), CHECKSUM)
    if isinstance(ch, bytes):
        return ord(ch) == CHECKSUM
    return ch == CHECKSUM



class WlDVLGenericError(Exception):
    """ Generic error """


class WlProtocolParseError(WlDVLGenericError):
    """ Error parsing sentence """


class WlProtocolChecksumError(WlProtocolParseError):
    """ Sentence checksum is invalid """


class WlProtocolParser(object):
    """
    Water Linked DVL protocol parser
    """
    def __init__(self):
        self.crc_func = None
        self.crc_func = crcmod.predefined.mkPredefinedCrcFun("crc-8")

    @staticmethod
    def do_format_checksum(checksum):
        if IS_PY2:
            return bytes("*{:02x}".format(checksum))

        return bytes("*{:02x}".format(checksum), "ascii")

    def checksum_for_buffer(self, data):
        if IS_PY2:
            csum = self.crc_func(bytes(data))
        else:
            csum = self.crc_func(data)
        return self.do_format_checksum(csum)

    def parse(self, sentence):
        sop = sentence[0]
        #print(sentence)
        if isinstance(sop, bytes):
            sop = ord(sop)
        if sop != SOP:
            return b'False'
            # This will swallow LF following a CR and garbage
            raise WlProtocolParseError("Missing SOP: Got {} Expected {}".format(sop, SOP))
        if len(sentence) < 3:  # Shortest possible command is 3, SOP+DIR+CMD
            raise WlProtocolParseError("Sentence is too short")

        direction = sentence[1]
        if isinstance(direction, bytes):
            direction = ord(direction)
        if direction not in [DIR_CMD, DIR_RESP]:
            raise WlProtocolParseError("Invalid direction {}: {}".format(direction, sentence))

        got_checksum = is_checksum(sentence[-3])
        csum = ""
        if got_checksum:
            csum = sentence[-3:]
            sentence = sentence[:-3]  # Remove checksum to ease further processing
            if csum != self.checksum_for_buffer(sentence):
                expect = self.checksum_for_buffer(sentence)
                raise WlProtocolChecksumError("Expected {} got {}".format(expect, csum))

        cmd = sentence[2]
        if isinstance(cmd, bytes):
            #print("Is cmd")
            cmd = ord(cmd)
        if cmd in ALL_VALID:
            fragments = sentence.split(b',')
            options = None
            if len(fragments) > 1:
                options = fragments[1:]

            return self.doDict(cmd, direction, options)

        return None

    def doDict(self, cmd, direction, options):
        result = {
            "time": float(options[0].decode('utf-8')),
            "vx": float(options[1].decode('utf-8')),
            "vy": float(options[2].decode('utf-8')),
            "vz": float(options[3].decode('utf-8')),
            "fom": float(options[4].decode('utf-8')),
            "altitude": float(options[5].decode('utf-8')),
            "valid": True if options[6].decode('utf-8') == 'y' else False,
        }
        return result


class WlDVLBase(object):
    """
    Water Linked DVL protocol parser base class
    """

    def __init__(self, iodev, debug=False):
        self._iodev = iodev
        self.parser = WlProtocolParser()

        self.payload_size = -1

        self._holdoff = 0
        self._buffer = bytearray()
        self.debug = debug

        self._rx_queue = list()

        self.oldString = ""

    # --------------------
    # Public API functions
    # --------------------
    

    def getData(self):
        oldString = self.oldString
        contain = 0
        raw_data = ""
        while contain == 0:
            raw_data = raw_data + self._iodev.read(1).decode("utf-8")
            if "\r\n" in raw_data:
                contain = 1
        raw_data = oldString + raw_data
        oldString = ""
        raw_data = raw_data.split("\r\n")
        oldString = raw_data[1]
        raw_data = raw_data[0] + "\r\n"
        self.oldString = oldString
        
        return raw_data

    def getPack(self):
        raw_data = 'False'
        while raw_data == 'False':
            raw_data = self._iodev.readline().decode("utf-8")
            print(raw_data)

        return raw_data

    def read(self):
        raw = ""
        nul_strip = ""
        while nul_strip == "":
            try:
                raw = ""
                raw = str(self.getPack().encode('utf-8'))                
                try:
                    nul_strip = "w" + raw.split("w")[1]
                    nul_strip = nul_strip.split("\r\n")[0]
                except Exception as err1:
                    pass
                    #print("Failed to fetch complete message: {}".format(err1))
                
                for c in nul_strip:
                    self._buffer.append(ord(c))
                #print(raw)
                #print(nul_strip)
                #print(self._buffer)
                if nul_strip != "":
                    packet = self.parser.parse(self._buffer)
                    self._buffer = bytearray()
                    return packet
            except WlProtocolParseError as err:
                log.warning("Connect error: {}".format(err))

        self._buffer = bytearray()

        return None


class WlDVL(WlDVLBase):
    """
    Water Linked DVL protocol parser
    """

    def __init__(self, device, baudrate=115200, debug=False):
        try:
            self._serial = serial.Serial(device, baudrate)
        except Exception as err:
            raise WlDVLGenericError("Error opening serial port {}".format(err))

        super(WlDVL, self).__init__(self._serial, debug=debug)
