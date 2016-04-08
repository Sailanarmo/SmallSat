#!/usr/bin/python

# Simple script to convert list of hex bytes on STDIN to a binary file for testing
# needs to be in the format of "0x01 0x02 0x03" etc

import sys
output =  open("test3.jpg", 'wb')

for line in sys.stdin:
    output.write(bytearray(int(i, 16) for i in line.strip().split(' ')))
    #print "hello"

output.close();

