# -*- coding: utf-8 -*-
import serial
import time
import sys
import linecache
import struct
import numpy as np
import threading
from plotIRSensor import *
from utils import linalg2_util as linalg

def PrintException():
    exc_type, exc_obj, tb = sys.exc_info()
    f = tb.tb_frame
    lineno = tb.tb_lineno
    filename = f.f_code.co_filename
    linecache.checkcache(filename)
    line = linecache.getline(filename, lineno, f.f_globals)
    print 'EXCEPTION IN ({}, LINE {} "{}"): {}'.format(filename, lineno, line.strip(), exc_obj)

class State:
    GO_TO_GOAL      = 0
    AVOID_OBSTACLE  = 1
    SLIDE_LEFT      = 2
    SLIDE_RIGHT     = 3
    GOAL_REACHED    = 4


class Roomba:

    SENSOR_DATA_LOCK = threading.Lock()

    V_MAX           = 200 #mm/s
    KP              = 10

    KP_OMEGA        = 5

    TURN_SPEED      = 200

    FOLLOW_DISTANCE = 0.5

    WHEEL_SEPARATION= 235   # En milimÃ¨tre
    WHEEL_DIAMETRE    = 72    # En milimÃ¨tre
    ROBOT_RADIUS    = 35    # En milimÃ¨tre

    SENSITIVE_ZONE  = 200
    DANGER_ZONE     = 50

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
    CMD_TO_OCTET    = { # packetIds : [octetsctet, 0(non signÃ©)/1(signÃ©)]
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
        self.state = State.GO_TO_GOAL
        self.goal = [0,0]
        self.Goal_Vector = [0,0,0]
        self.position = [0,0,0] #x, y, theta (en milimetre)
        self.last_encodeur = [] #[left, right]
        self.minAbsDistanceToGoal = sys.maxint;

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
        time.sleep(0.2)

    def safeMode(self):
        self.send([self.CMD_MODE_SAFE])
        time.sleep(0.2)

    def fullMode(self):
        self.send([self.CMD_MODE_FULL])
        time.sleep(0.2)

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
        #print trame

        checksum = (19 + n + sum(bytearray(trame))) & 0xFF
        if checksum == 0:
            i = 0;
            while i < n :
                cmd = ord(trame[i])
                #print "cmd = ", cmd
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

    def convertDistance2Ir(self, distance):
        return 2436 * np.exp(-0.024 * distance)

    def convertIR2Distance(self, irValue):
        return -42.16 * np.log(irValue) + 329.15

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

#     def turnTo(self, newTheta):
#         theta = self.supervisor.estimated_pose().theta
#         e = ((theta_d - theta + np.pi) % (2*np.pi)) - np.pi
#         omega = self.k_p * e
#         self.supervisor.set_outputs( 1.0, omega )
#         self.diffMove(np.sign(delta)*self.TURN_SPEED,-1*np.sign(delta)*self.TURN_SPEED)
#         time.sleep(t)
#         self.diffMove(0,0)

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

    def moveToGoal(self, xPos, yPos): #xPos et yPos en mètre

        thetaDelta = ((self.Goal_Vector[2] - self.position[2] + np.pi) % (2*np.pi)) - np.pi
        # Calcul omega et v
        omega = self.KP * thetaDelta
        v = self.V_MAX / ( abs( omega ) + 1 )**0.5
        self.uniMove(v, omega)

    def avoidObtacles(self):
        n = self.RC2_SENSOR_POSES
        distanceSensors = self.getIrData()

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

        self.uniMove(v,omega)


    def getIrData(self):
        self.SENSOR_DATA_LOCK.acquire()
        irSensors = [  self.SENSOR_DATA[46],
                       self.SENSOR_DATA[47],
                       self.SENSOR_DATA[48],
                       self.SENSOR_DATA[49],
                       self.SENSOR_DATA[50],
                       self.SENSOR_DATA[51]]
        self.SENSOR_DATA_LOCK.release()
        return irSensors

    def followWall(self):
        wall_surface =             [ [ 1.0, 0.0 ], [ 1.0, 0.0 ] ]  # the followed surface, in robot space
        parallel_component =       [ 1.0, 0.0 ]
        perpendicular_component =  [ 1.0, 0.0 ]
        distance_vector =          [ 1.0, 0.0 ]
        fw_heading_vector =        [ 1.0, 0.0 ]

        if self.state == State.SLIDE_LEFT:
            self.SENSOR_DATA_LOCK.acquire()
            sensorDistances = [self.SENSOR_DATA[46],
                               self.SENSOR_DATA[47],
                               self.SENSOR_DATA[48]]
            self.SENSOR_DATA_LOCK.release()
            sensorsPlacement = self.RC2_SENSOR_POSES[0:3]
        else:
            self.SENSOR_DATA_LOCK.acquire()
            sensorDistances = [self.SENSOR_DATA[51],
                               self.SENSOR_DATA[20],
                               self.SENSOR_DATA[49]]
            self.SENSOR_DATA_LOCK.release()
            sensorsPlacement = self.RC2_SENSOR_POSES[5:2:-1]


        sensorDistances, indices = zip( *sorted( zip(
                                            sensorDistances,
                                            [0, 1, 2,]
                                    )))
        d1, d2 = sensorDistances[0:2]
        i1, i2 = indices[0:2]
        sensor1Pos, sensor1Theta = sensorsPlacement[i1][0] , sensorsPlacement[i1][1]
        sensor2Pos, sensor2Theta = sensorsPlacement[i2][0] , sensorsPlacement[i2][1]
        p1, p2 = [ d1, 0.0 ], [ d2, 0.0 ]
        p1 = linalg.rotate_and_translate_vector( p1, sensor1Pos, sensor1Theta )
        p2 = linalg.rotate_and_translate_vector( p2, sensor2Pos, sensor2Theta )

        wall_surface = [ p2, p1 ]
        parallel_component = linalg.sub( p2, p1 )
        distance_vector = linalg.sub( p1, linalg.proj( p1, parallel_component ) )
        unit_perp = linalg.unit( distance_vector )
        distance_desired = linalg.scale( unit_perp, self.FOLLOW_DISTANCE )
        perpendicular_component = linalg.sub( distance_vector, distance_desired )
        fw_heading_vector = linalg.add( parallel_component, perpendicular_component )

        theta = np.arctan2(fw_heading_vector[1], fw_heading_vector[0])

        omega = self.KP_OMEGA*theta

        v = self.V_MAX / ( abs( omega ) + 1 )**0.5

        self.uniMove(v,omega)

    def goalUpdate(self):
        # Calcul vector goal robot frame
        self.Goal_Vector[0] = self.goal[0] - self.position[0]
        self.Goal_Vector[1] = self.goal[1] - self.position[1]
        self.Goal_Vector[2] = np.arctan2(self.Goal_Vector[1],self.Goal_Vector[0])

    def getInput(self):

        self.isInDanger = False         # X
        self.obstacleDetected = False   # X
        self.isSafe = False             # X
        self.isSlidingLeft = False
        self.isSlidingRight = False
        self.isGoalReached = False      # X
        self.progressMade = False       # X

        absDistanceToGoal = np.sqrt(self.Goal_Vector[0]**2 + self.Goal_Vector[1]**2)

        #RC2_SENSOR_POSES
        distanceSensors = self.getIrData()

        for ir in distanceSensors:
            distance = self.convertIR2Distance(ir)
            if( distance >= self.SENSITIVE_ZONE ):
                self.isSafe = True
            if( distance < self.SENSITIVE_ZONE ):
                self.obstacleDetected = True
            if( distance < self.DANGER_ZONE ):
                self.isInDanger = True
        if ( -5 < self.Goal_Vector[0] and self.Goal_Vector[0] < 5 and -5 < self.Goal_Vector[1] and self.Goal_Vector[1] < 5):
            self.isGoalReached = True
        if absDistanceToGoal < self.minAbsDistanceToGoal:
            self.progressMade = True
            self.minAbsDistanceToGoal = absDistanceToGoal


    def doGoToGoal(self):

        if(self.isGoalReached):
            self.state = State.GOAL_REACHED
        elif(self.isInDanger):
            self.state = State.AVOID_OBSTACLE
        elif(self.obstacleDetected and self.isSlidingLeft and not self.isSlidingRight):
            self.state = State.SLIDE_LEFT
        elif(self.obstacleDetected and self.isSlidingRight and not self.isSlidingLeft):
            self.state = State.SLIDE_RIGHT

        #action
        pass

    def doAvoidObstacle(self):

        if(self.isGoalReached):
            self.state = State.GOAL_REACHED
        elif(not self.isInDanger and self.isSlidingLeft and not self.isSlidingRight):
            self.state = State.SLIDE_LEFT
        elif(not self.isInDanger and self.isSlidingRight and not self.isSlidingLeft):
            self.state = State.SLIDE_RIGHT
        elif(not self.isSlidingLeft and not self.isSlidingRight):
            self.state = State.GO_TO_GOAL

        #action
        pass

    def doSlideLeft(self):

        if(self.isGoalReached):
            self.state = State.GOAL_REACHED
        elif(self.isInDanger):
            self.state = State.AVOID_OBSTACLE
        elif(self.progressMade and not self.isSlidingLeft):
            self.state = State.GO_TO_GOAL

        #action

        pass

    def doSlideRight(self):

        if(self.isGoalReached):
            self.state = State.GOAL_REACHED
        elif(self.isInDanger):
            self.state = State.AVOID_OBSTACLE
        elif(self.progressMade and not self.isSlidingRight):
            self.state = State.GO_TO_GOAL

        #action
        pass

    def updateState(self):
        if(self.state == State.GO_TO_GOAL):
            self.doGoToGoal()
        elif(self.state == State.AVOID_OBSTACLE):
            self.doAvoidObstacle()
        elif(self.state == State.SLIDE_LEFT):
            self.doSlideLeft()
        elif(self.state == State.SLIDE_RIGHT):
            self.doSlideLeft()

    def update(self):
        self.goalUpdate()
        self.getInput()
        self.updateState()


def test():
    r = Roomba('COM24', 115200)
    s = Stream(r)
    try:
        r.fullMode()
        print "mode : {0}".format(r.getMode())

        s.start()
        time.sleep(1)

        while True:
            r.getInput()
            print "running"
            print "obstacle ? {}".format(r.obstacleDetected)
            print "danger ? {}".format(r.isInDanger)
            time.sleep(1)

        r.disconnect()

    except:
        PrintException()
        s.kill()
        r.disconnect()


if __name__ == '__main__':
    test()
