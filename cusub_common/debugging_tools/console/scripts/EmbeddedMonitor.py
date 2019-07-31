#!/usr/bin/env python
import serial
import struct
import binascii
import subprocess
import time

ser = serial.Serial('/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN05ZN8B-if00-port0', 115200)

data = b''

last_alert = None

def alert_sub(voltage):
	global last_alert
	if last_alert is None or (time.time() - last_alert) > 15:
		subprocess.call(["wall", "\"Battery looowwwwww... %.2f\"" % voltage])
		last_alert = time.time()

def shutdown_sub():

	subprocess.call(["wall", "\"SHUTDOWN | SHUTDOWN | SHUTDOWN\""])
	time.sleep(5)
	subprocess.call(["rosnode", "kill", "-a"])
	time.sleep(120)

while(1):

	data_byte = ser.read(1)
	if ord(data_byte[0]) == 0xC0:
		if len(data) > 0:
			data = data.replace('\xDB\xDC', '\xC0').replace('\xDB\xDD', '\xDB')

			if   ord(data[0]) == 0xDD: # MOTOR_CURRENT
				m = []
				for i in range(8):
					m.append(0.0)
                                m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7] = struct.unpack("<HHHHHHHH", data[1:])
				for i in range(8):
	                                m_current = ((1.2*m[i]/(2**14))/20/0.0003)
        	                        #print("M%d: %.2f A" % (i, m_current))
				#print(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7])

			elif ord(data[0]) == 0xEE: # POWER_CURRENT
				p1, p2, p3, p4, p5, p6 = struct.unpack("<HHHHHH", data[1:])
				# 3.3V 9V 12V 19V 48V VBAT
				#print(p1, p2, p3, p4, p5, p6)
				#print(" 9V: %.2f A" % ((1.2*p2/(2**14))/20/0.006) )
				#print("19V: %.2f A" % ((1.2*p4/(2**14))/20/0.001) )
				#print("48V: %.2f A" % ((1.2*p4/(2**14))/20/0.010) )
				bat_voltage = (14.1*1.2*p6/(2**14))
				print("BAT: %.2fv" % bat_voltage)
				if(bat_voltage < 15.0):
					alert_sub(bat_voltage)
				elif(bat_voltage < 14.2):
					shutdown_sub()

			#print(binascii.hexlify(data))
			# Deal with it
		data = b''
	else:
		data += data_byte

ser.close()
