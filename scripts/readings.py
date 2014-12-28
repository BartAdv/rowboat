import serial
import msvcrt
import struct

out = open("readings.csv", 'w')

s = serial.Serial(2, 115200)
s.readline()
print "let's go"

try:
    while True:
        pwm, = struct.unpack('h', s.read(2))
        t, = struct.unpack('I', s.read(4))
        out.write("{0};{1}\n".format(pwm, t))
except KeyboardInterrupt:             
    out.close()
    s.close()
