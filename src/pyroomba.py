import serial
import time

class Roomba:

    CMD_RESET       = 7
    CMD_START       = 128
    CMD_STOP        = 173
    CMD_POWER_DOWN  = 133
    CMD_MODE_SAFE   = 131
    CMD_MODE_FULL   = 132
    CMD_LED         = 139
    CMD_SONG_SET    = 140
    CMD_SONG_PLAY   = 141
    CMD_QUERY_LIST  = 149
    CMD_STREAM      = 148

    PKT_ID_MODE     = 35

    MODE_PASSIVE    = 1
    MODE_SAFE       = 2
    MODE_FULL       = 3
    
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

    
r = Roomba('COM24', 115200)
try:
    r.fullMode()
    
    print "mode : {0}".format(r.getMode())

    #r.setSong(1, [61,50,62,50,63,50])
    #r.playSong(1)
    
    r.setStream([r.PKT_ID_MODE])
    while True:
        if ord(r.receive(1)) == 19:
            print 'new frame : ',
            n = ord(r.receive(1))
            data = r.receive(n+1)
            for i in xrange(0,n-1,2):
                print 'data {0} = {1}; '.format(ord(data[i]), ord(data[i+1])),
            checksum = (19 + n + sum(bytearray(data))) & 0xFF
            if checksum != 0:
                print '[bad CRC : {0}]'.format(checksum),
            print '\n\r'
                
    
    r.disconnect()
    
except Exception as e:
    print "Catch exception : {0}".format(str(e))
    r.disconnect()



