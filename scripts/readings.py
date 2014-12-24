import serial
import msvcrt
import struct

out = open("readings.csv", 'w')

s = serial.Serial(2, 115200)
s.readline()
print "let's go"

try:
    while True:
        t, = struct.unpack('I', s.read(4))
        v, = struct.unpack('h', s.read(2))
        out.write("{0};{1}\n".format(t, v))
except KeyboardInterrupt:             
    out.close()
    s.close()
