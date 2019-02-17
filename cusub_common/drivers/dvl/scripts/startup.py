import signal
import sys
import getpass
import sys
import telnetlib
import rospy

HOST = "10.0.0.3"					#need to make this a static IP address
tn = telnetlib.Telnet(HOST, "9000")

def DVL():
	print tn.read_until("Username:")
	tn.write("nortek\r\n")
	print tn.read_until("Password:")
	tn.write("nortek\r\n")
	print tn.read_until("Interface")
	tn.write("\x03\r\n")
	print tn.read_until("OK")
	tn.write("MC\r\n")
	print tn.read_until("OK")
	tn.write("POWERDOWN\r\n")
	print tn.read_until("OK")
	tn.close()
	print("Shut down at startup")
	sys.exit(0)

if __name__ == '__main__':
	DVL()
