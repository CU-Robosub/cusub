#!/usr/bin/env python
import serial, string



if __name__ == '__main__':
        ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.02)
        with open("2D_cal_commands.txt") as f:
            for line in f:
                print(line)
                ser.write(line)
                ser.write("\r\n")
                print(ser.read(1000))

        ser.close()
