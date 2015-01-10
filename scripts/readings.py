import serial
import msvcrt
import struct

out = open("readings.csv", 'w')

s = serial.Serial(2, 115200)
s.readline()
print "let's go"

def readLong():
    v, = struct.unpack('I', s.read(4))
    return v

def readInt():
    v, = struct.unpack('h', s.read(2))
    return v
    
try:
    while True:
        pwm = readInt()
        tt = readLong()
        out.write("{0};{1}\n".format(pwm, tt))
except KeyboardInterrupt:             
    out.close()
    s.close()
