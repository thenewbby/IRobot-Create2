#!/usr/bin/python
from matplotlib import pyplot as plt
import numpy as np
import serial
import time


def plotData(data=(20, 35, 30, 35, 27, 18)):
    ind = np.arange(len(data))
    #data = (20, 35, 30, 35, 27, 18)
    plt.bar(ind, data, 0.35,color='r',)
    plt.show()

def main():
    ser = serial.Serial('/dev/ttyUSB0',115200)
    #ser.open()

    c = bytearray([chr(148),chr(6), chr(46), chr(47), chr(48), chr(49), chr(50), chr(51)])
    # ser.write(bytes('\x80'))
    song = bytearray([chr(140),chr(2), chr(31), chr(100), chr(50), chr(100), chr(100), chr(150)])
    # ser.write(a)
    n = ser.write(c)
    # print song
    # ser.write(song)

    # ser.write(bytes('\x23'))
    # s = ser.read()


    # try:
    #     if ser is not None:
    #         # ser.write(bytes('\x80'))
    #         # ser.write(bytes(chr(132)))
    #     else:
    #         print "pas de co"
    # except Exception as e:
    #     print "NOPE!"
    # ser.write(bytearray([chr(139),chr(2),chr(0),chr(0)]))
    # ser.write(chr(173))
    # n = ser.write(song)
    # print n
    # s = ser.read( 88 )
    # print s
    while 1:
        s = ser.read( 88 )
        print s
        time.sleep(0.02)



if __name__ == '__main__':
    main()
