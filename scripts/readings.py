import serial
import struct
import numpy

out = open("readings.csv", 'w')

s = serial.Serial("/dev/ttyUSB0", 115200)
s.readline()
print("let's go")

def readLong():
    v, = struct.unpack('I', s.read(4))
    return v

def readInt():
    v, = struct.unpack('h', s.read(2))
    return v

def readFloat():
    v, = numpy.frombuffer(s.read(2), dtype=numpy.float16)
    return v

try:
    while True:
        x, y, z = readFloat(), readFloat(), readFloat()
        ln = "{0};{1};{2}\n".format(x, y, z)
        print(ln)
        out.write(ln)
except KeyboardInterrupt:             
    out.close()
    s.close()
