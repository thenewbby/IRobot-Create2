# -*- coding: utf-8 -*-
import serial
import time
import sys
import linecache
import struct
import numpy as np
import threading
from utils import linalg2_util as linalg

def PrintException():
    exc_type, exc_obj, tb = sys.exc_info()
    f = tb.tb_frame
    lineno = tb.tb_lineno
    filename = f.f_code.co_filename
    linecache.checkcache(filename)
    line = linecache.getline(filename, lineno, f.f_globals)
    print 'EXCEPTION IN ({}, LINE {} "{}"): {}'.format(filename, lineno, line.strip(), exc_obj)

class Roomba:

    SENSOR_DATA_LOCK = threading.Lock()

    position = [0,0,0] #x, y, theta (en milimetre)
    last_encodeur = [] #[left, right]
    # last_time_pos =

    V_MAX           = 200 #mm/s
    KP              = 10
    KP_OMEGA        = 5

    WHEEL_SEPARATION= 235   # En milimètre
    WHEEL_DIAMETRE    = 72    # En milimètre
    ROBOT_RADIUS    = 35    # En milimètre

    CMD_RESET       = 7
    CMD_START       = 128
    CMD_STOP        = 173
    CMD_POWER_DOWN  = 133
    CMD_MODE_SAFE   = 131
    CMD_MODE_FULL   = 132
    CMD_LED         = 139
    CMD_SONG_SET    = 140
    CMD_SONG_PLAY   = 141
    CMD_DRIVE       = 145
    CMD_QUERY_LIST  = 149
    CMD_STREAM      = 148

    PKT_ID_MODE     = 35

    MODE_PASSIVE    = 1
    MODE_SAFE       = 2
    MODE_FULL       = 3

    ENCODER_MAX_VALUE = 65535

    SENSOR_GAIN = [9.27, 4.82, 1.76, 1.76, 4.82, 9.27]

    SENSOR_DATA = {}
# LISTE NON COMPLETE
    CMD_TO_OCTET    = { # packetIds : [octetsctet, 0(non signé)/1(signé)]
                        7 : [1,0],  #Bumps and Wheel Drops
                        8 : [1,0],  #Wall
                        9 : [1,0],  #Cliff Left
                        10 : [1,0], #Cliff Front Left
                        11 : [1,0], #Cliff Front Right
                        12 : [1,0], #Cliff Right
                        13 : [1,0], #Virtual Wall
                        14 : [1,0], #Wheel Overcurrents
                        17 : [1,0], #Infrared CharacterOmni
                        52 : [1,0], #Infrared Character Left
                        53 : [1,0], #Infrared Character Right
                        18 : [1,0], #Buttons
                        19 : [2,1], #Distance
                        20 : [2,1], #Angle
                        35 : [1,0], #OI Mode
                        38 : [1,0], #Number of Stream Packets
                        39 : [2,1], #Requested Velocity
                        40 : [2,1], #Requested Radius
                        41 : [2,1], #Requested Right Velocity
                        42 : [2,1], #Requested Left Velocity
                        43 : [2,0], #LeftEncoder Counts
                        44 : [2,0], #RightEncoder Counts
                        45 : [1,0], #Light Bumper
                        # Groupe commande 106 (46 - 51)
                        46 : [2,0], #Light Bump Left Signal
                        47 : [2,0], #Light Bump Front Left Signal
                        48 : [2,0], #Light Bump Center Left Signal
                        49 : [2,0], #Light Bump Center Right Signal
                        50 : [2,0], #Light Bump Front Right Signal
                        51 : [2,0] #Light Bump Right Signal
                      }
    RC2_SENSOR_POSES = [[ [0.147,  0.315],  1.134 ], # [x, y], theta_radian
                       [  [0.301,  0.174], 0.523  ],
                       [  [0.346,  0.036],  0.104  ],
                       [  [0.346, -0.036], -0.104  ],
                       [  [0.301, -0.174], -0.523  ],
                       [  [0.147, -0.315], -1.134  ]]

    def __init__(self, port, baudrate):
        self.s = None
        self.connect(port, baudrate)

    def connect(self, port, baudrate):
        self.s = serial.Serial(port, baudrate, timeout = 1)
        self.send([self.CMD_START])
        time.sleep(0.2)

    def disconnect(self):
        self.send([self.CMD_STOP])
        self.s.close()

    def send(self, command):
        try:
            # print command
            self.s.write(command)
        except IOError as e:
            print "Roomba I/O error({0}) : {1}".format(e.errno, e.strerror)

    def receive(self, byteCount):
        try:
            return self.s.read(byteCount)
        except IOError as e:
            print "Roomba I/O error({0}) : {1}".format(e.errno, e.strerror)

    def passiveMode(self):
        self.send([self.CMD_START])
        time.sleep(0.04)

    def safeMode(self):
        self.send([self.CMD_MODE_SAFE])
        time.sleep(0.04)

    def fullMode(self):
        self.send([self.CMD_MODE_FULL])
        time.sleep(0.04)

    def getMode(self):
        self.send([self.CMD_QUERY_LIST, 1, self.PKT_ID_MODE])
        return ord(self.receive(1))

    def setSong(self, songId, notes):
        assert isinstance(notes, list), 'notes must be a list'
        data = [self.CMD_SONG_SET, songId, len(notes)/2]
        data.extend(notes)
        self.send(data)

    def playSong(self, songId):
        self.send([self.CMD_SONG_PLAY, songId])

    # def getData(self, packetIds): #NE MARCHE PAS
    #     assert isinstance(packetIds, list), 'packetIds must be a list'
    #     data = [self.CMD_QUERY_LIST, len(packetIds)]
    #     data.extend(packetIds)
    #     self.send(data)
    #     rec = []
    #     octets = 0
    #     for pck in packetIds:
    #         octets += self.CMD_TO_OCTET[pck][0]
    #         # print octets
    #     temp = self.receive(octets)
    #     print temp
    #     print packetIds
    #     i = 0
    #     for pck in packetIds:
    #         octets ,sign = self.CMD_TO_OCTET[pck]
    #         if octets == 1:
    #             print ord(temp[i:i+1])
    #             # rec.append(ord(temp[i:i+1]))
    #         else:
    #             if sign == 0:
    #                 sType = '>H'
    #             else:
    #                 sType = '>h'
    #             print struct.unpack(sType,temp[i:i+octets])[0]
    #             # rec.append(struct.unpack(sType,temp)[0])
    #         i+= octets
    #     return rec

    def setStream(self, packetIds):
        assert isinstance(packetIds, list), 'packetIds must be a list'
        data = [self.CMD_STREAM, len(packetIds)]
        data.extend(packetIds)
        self.send(data)

    def getDataSTream(self): #Verifier avant d'appeler si le premier hexa est 19
        n = ord(self.receive(1))
        trame = self.receive(n+1)

        checksum = (19 + n + sum(bytearray(trame))) & 0xFF
        if checksum == 0:
            i = 0;
            while i < n :
                    cmd = ord(trame[i])
                    octets, sign = self.CMD_TO_OCTET[cmd]
                    if octets == 1:
                        data = ord(trame[i+1:i+2])
                    else:
                        temp = self.receive(octets)
                        if sign == 0:
                            sType = '>H'
                        else:
                            sType = '>h'
                        data = struct.unpack(sType, trame[i+1:i+1+octets])[0]
                    self.SENSOR_DATA_LOCK.acquire()
                    self.SENSOR_DATA[cmd] = data
                    self.SENSOR_DATA_LOCK.release()
                    i += 1 + octets
        else:
            print "trame corrompue"

    def unicycleToDifferential(self, v, omega):
        R = self.WHEEL_SEPARATION
        L = self.WHEEL_DIAMETRE
        rSpeed = ( (2.0 * v) + (omega*L) ) / (2.0 )
        lSpeed = ( (2.0 * v) - (omega*L) ) / (2.0 )
        # print "lSpeed: " + str(lSpeed) + " rSpeed: " + str(rSpeed)
        return lSpeed, rSpeed

    def positionUpdate(self):
        self.SENSOR_DATA_LOCK.acquire()
        try:
            left_encoder  = self.SENSOR_DATA[43]
            right_encoder = self.SENSOR_DATA[44]
        except:
            PrintException()
        self.SENSOR_DATA_LOCK.release()
        # print last_encodeur
        left_lenth  = left_encoder - self.last_encodeur[0]
        right_lenth = right_encoder - self.last_encodeur[1]

        self.last_encodeur[0] = left_encoder
        self.last_encodeur[1] = right_encoder

        if left_lenth > 10000:
            left_lenth -= self.ENCODER_MAX_VALUE
        elif left_lenth < -10000:
            left_lenth += self.ENCODER_MAX_VALUE

        if right_lenth > 10000:
            right_lenth -= self.ENCODER_MAX_VALUE
        elif right_lenth < -10000:
            right_lenth += self.ENCODER_MAX_VALUE


        left_lenth *= (np.pi * self.WHEEL_DIAMETRE)/508.8
        right_lenth *= (np.pi * self.WHEEL_DIAMETRE)/508.8

        lenth_sum = left_lenth + right_lenth

        average_lenth = (lenth_sum) /2

        angle = self.position[2]


        self.position[0] += average_lenth * np.cos(angle)
        self.position[1] += average_lenth * np.sin(angle)
        self.position[2] += (right_lenth - left_lenth) /  self.WHEEL_SEPARATION
        self.position[2] = ((self.position[2] + np.pi) % (2*np.pi)) - np.pi

    def uniMove(self, v, omega):
        lSpeed, rSpeed = self.unicycleToDifferential(v, omega)
        self.diffMove(lSpeed, rSpeed)

    def diffMove(self,lSpeed, rSpeed):
        r = struct.pack('>h',rSpeed)
        l = struct.pack('>h',lSpeed)

        data = [self.CMD_DRIVE, r[0:1], r[1:2], l[0:1], l[1:2]]
        self.send(data)

    def stopTurnTo(self,newTheta):
        self.diffMove(0,0)
        newTheta = np.arctan2(np.sin(newTheta),np.cos(newTheta))
        while True:
                delta = newTheta - self.position[2]
                omega = self.KP_OMEGA * delta
                self.uniMove(0,omega)
                if -0.087 < delta and delta < 0.087:
                    break
        self.diffMove(0,0)

    def moveToPoint(self, xPos, yPos): #xPos et yPos en mètre
        Goal_Vector = [100,100,0]
        xPos *= 1000
        yPos *= 1000
        while (-5 < Goal_Vector[0]) and (Goal_Vector[0] > 5) and (-5 < Goal_Vector[1]) and (Goal_Vector[1] > 5): # Condition d'arrete : arrivé au point (vecteur goal proche de zéro)
            # Calcul vector goal robot frame
            Goal_Vector[0] = xPos - self.position[0]
            Goal_Vector[1] = yPos - self.position[1]
            Goal_Vector[2] = np.arctan2(Goal_Vector[1],Goal_Vector[0])

            # Calcul de l'angle d'erreur
            thetaDelta = ((Goal_Vector[2] - self.position[2] + np.pi) % (2*np.pi)) - np.pi
            # Calcul omega et v
            omega = self.KP * thetaDelta
            v = self.V_MAX / ( abs( omega ) + 1 )**0.5
            self.uniMove(v, omega)
        self.diffMove(0,0)

    def avoidObtacles(self):
        n = self.RC2_SENSOR_POSES
        self.SENSOR_DATA_LOCK.acquire()
        distanceSensors = [self.SENSOR_DATA[46],
                           self.SENSOR_DATA[47],
                           self.SENSOR_DATA[48],
                           self.SENSOR_DATA[49],
                           self.SENSOR_DATA[50],
                           self.SENSOR_DATA[51]]
        self.SENSOR_DATA_LOCK.release()

        obstacle_vectors = [[ 0.0, 0.0 ] * n]
        heading_vector   = [ 0.0, 0.0 ]
        for i in range(n):
            obstacle_vectors[i] = distanceSensors[i]
            obstacle_vectors[i] = linalg.rotate_and_translate_vector( obstacle_vectors[i], self.RC2_SENSOR_POSES[i][1], self.RC2_SENSOR_POSES[i][0] )
            heading_vector      = linalg.add( heading_vector,
                                             linalg.scale( obstacle_vectors[i], self.SENSOR_GAIN[i] ) )

        theta = np.arctan2(heading_vector[1], heading_vector[0])

        omega = self.KP_OMEGA*theta

        v = self.V_MAX / ( abs( omega ) + 1 )**0.5

        r.uniMove(v,omega)


def test(arg):
    r = Roomba('COM24', 115200)
    try:
        r.fullMode()

        print "mode : {0}".format(r.getMode())

        #r.setSong(1, [61,50,62,50,63,50])
        #r.playSong(1)

        r.setStream([46])
        while True:
            if ord(r.receive(1)) == 19:
                r.getDataSTream();
                print "data : {0}".format(r.SENSOR_DATA[46])


        r.disconnect()

    except:
        PrintException()
        r.disconnect()


if __name__ == '__main__':
    test()
