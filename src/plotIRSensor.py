#!/usr/bin/python
from matplotlib import pyplot as plt
import numpy as np
import serial
import time
from pyroomba import Roomba
import threading

def plotData(data):
    ind = np.arange(len(data))
    #data = (20, 35, 30, 35, 27, 18)
    plt.bar(ind, data, 0.35,color='r',)
    plt.show()
    threading.Timer(1, plotData,args=[r.SENSOR_DATA[46],r.SENSOR_DATA[47],r.SENSOR_DATA[48],r.SENSOR_DATA[49],r.SENSOR_DATA[50],r.SENSOR_DATA[51]])

def read():
    while True:
        if ord(r.receive(1)) == 19:
            r.getDataSTream()


def main():
    r = Roomba('/dev/ttyUSB0',115200)

    r.fullMode()

    r.setStream([46,47,48,49,50,51])

    threading.Timer(1, plotData,args=[r.SENSOR_DATA[46],r.SENSOR_DATA[47],r.SENSOR_DATA[48],r.SENSOR_DATA[49],r.SENSOR_DATA[50],r.SENSOR_DATA[51]])

    while True:
        if ord(r.receive(1)) == 19:
            r.getDataSTream()


if __name__ == '__main__':
    main()
