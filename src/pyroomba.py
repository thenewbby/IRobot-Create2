import serial
import time
import sys
import linecache
import struct
import numpi as np

def PrintException():
    exc_type, exc_obj, tb = sys.exc_info()
    f = tb.tb_frame
    lineno = tb.tb_lineno
    filename = f.f_code.co_filename
    linecache.checkcache(filename)
    line = linecache.getline(filename, lineno, f.f_globals)
    print 'EXCEPTION IN ({}, LINE {} "{}"): {}'.format(filename, lineno, line.strip(), exc_obj)

class Roomba:

    position = [0,0,0] #x, y, theta
    last_encodeur = []
    last_time_pos =

    WHEEL_SEPARATION  = 0.235
    WHEEL_RADIUS    = 0.072
    ROBOT_RADIUS    = 0.35

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

    SENSOR_DATA = {}

    CMD_TO_OCTET    = {
                        7 : 1,  #Bumps and Wheel Drops
                        8 : 1,  #Wall
                        9 : 1,  #Cliff Left
                        10 : 1, #Cliff Front Left
                        11 : 1, #Cliff Front Right
                        12 : 1, #Cliff Right
                        13 : 1, #Virtual Wall
                        14 : 1, #Wheel Overcurrents
                        17 : 1, #Infrared CharacterOmni
                        52 : 1, #Infrared Character Left
                        53 : 1, #Infrared Character Right
                        18 : 1, #Buttons
                        19 : 2, #Distance
                        20 : 2, #Angle
                        35 : 1, #OI Mode
                        38 : 1, #Number of Stream Packets
                        39 : 2, #Requested Velocity
                        40 : 2, #Requested Radius
                        41 : 2, #Requested Right Velocity
                        42 : 2, #Requested Left Velocity
                        43 : 2, #LeftEncoder Counts
                        44 : 2, #RightEncoder Counts
                        45 : 1, #Light Bumper
                        # Groupe commande 106 (46 - 51)
                        46 : 2, #Light Bump Left Signal
                        47 : 2, #Light Bump Front Left Signal
                        48 : 2, #Light Bump Center Left Signal
                        49 : 2, #Light Bump Center Right Signal
                        50 : 2, #Light Bump Front Right Signal
                        51 : 2 #Light Bump Right Signal
                      }

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

    def safeMode(self):
        self.send([self.CMD_MODE_SAFE])

    def fullMode(self):
        self.send([self.CMD_MODE_FULL])

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

    def setStream(self, packetIds):
        assert isinstance(packetIds, list), 'packetIds must be a list'
        data = [self.CMD_STREAM, len(packetIds)]
        data.extend(packetIds)
        self.send(data)

    def getDataSTream(self):
        n = ord(r.receive(1))
        trame = self.receive(n+1)

        checksum = (19 + n + sum(bytearray(trame))) & 0xFF
        if checksum == 0:
            i = 0;
            while i < n :
                    cmd = ord(trame[i])
                    octets = self.CMD_TO_OCTET[cmd]
                    data = struct.unpack('>H', trame[i+1:i+1+octets])[0]
                    self.SENSOR_DATA[cmd] = data
                    i += 1 + octets
        else:
            print "trame corrompue"


    def positionUpdate(self):
        left_lenth  = self.SENSOR_DATA[43] - self.last_encodeur[0]
        right_lenth = self.SENSOR_DATA[44] - self.last_encodeur[1]

        self.last_encodeur[0] = self.SENSOR_DATA[43]
        self.last_encodeur[1] = self.SENSOR_DATA[44]

        if left_lenth > 10000:
            left_lenth -= ENCODER_MAX_VALUE
        elif left_lenth < -10000:
            left_lenth += ENCODER_MAX_VALUE

        if right_lenth > 10000:
            right_lenth -= ENCODER_MAX_VALUE
        elif right_lenth < -10000:
            right_lenth += ENCODER_MAX_VALUE


        left_lenth *= (np.pi * WHEEL_RADIUS)/508.8
        right_lenth *= (np.pi * WHEEL_RADIUS)/508.8

        lenth_sum = left_lenth + right_lenth

        average_lenth = (lenth_sum) /2

        angle = self.position[2]


        self.position[0] += average_lenth * np.cos(angle)
        self.position[1] += average_lenth * np.sin(angle)
        self.position[2] += lenth_sum /  WHEEL_SEPARATION
        self.position[2] = self.position[2] % (2*np.pi)

    def move(self,rSpeed, lSpeed):

        data = [self.CMD_DRIVE, rSpeed, lSpeed]
        self.send(data)


    def stopTurnTo(self,newTheta):
        turnSpeed = 200
        self.move(0,0)
        delta = self.position[2] - newTheta
        if delta < -np.pi:
            delta += np.pi
        dist = (delta * WHEEL_SEPARATION) / 2
        time = abs(delta) / turnSpeed
        self.move(np.sign(delta)*turnSpeed,-1*np.sign(delta)*turnSpeed)
        sleep(time)
        self.move(0,0)


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
