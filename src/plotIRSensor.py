#!/usr/bin/python
import numpy as np
import serial
import time
from pyroomba import Roomba
import threading

def odo(r):
    while 1:
        b = r.receive(1)
        if ord(b) == 19:
            self.getDataSTream() 
def goToPoint(r):


def testMove():
    threads = []
    r = Roomba('/dev/ttyUSB0',115200)

    r.fullMode()

    threads.append(threading.Thread(Target = ,args = r))
    threads.append(threading.Thread(Target = ,args = r))

    for t in threads:
        t.start()


def test():
    r = Roomba('/dev/ttyUSB0',115200)

    r.fullMode()

    r.setSong(1, [61,50,62,50,63,50])
    r.playSong(1)

    r.disconnect()


    # r.setStream([46,47,48,49,50,51])
    #
    # threading.Timer(1, plotData,args=[r.SENSOR_DATA[46],r.SENSOR_DATA[47],r.SENSOR_DATA[48],r.SENSOR_DATA[49],r.SENSOR_DATA[50],r.SENSOR_DATA[51]])
    #
    # while True:
    #     if ord(r.receive(1)) == 19:
    #         r.getDataSTream()


if __name__ == '__main__':
    test()
