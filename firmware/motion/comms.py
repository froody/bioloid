import serial

ser = serial.Serial("/dev/tty.KeySerial1", 57600, timeout=0)
ser.open()

if not ser.isOpen():
	sys.exit()
		
print ser.readall()

def servos():
	ser.write(chr(3));
	print ser.readall()

def setservo(servo,angle, speed):
	ser.write(chr(4) + chr(servo)
			+ chr(angle & 0xff)
			+ chr(angle >> 8)
			+ chr(speed & 0xff)
			+ chr(speed>>8))
	print ser.readall()

def page(p):
	ser.write(chr(1)+chr(p))
	print ser.readall()

def pose(p,x):
	ser.write(chr(2)+chr(p)+chr(x))
	print ser.readall()
