# #!/usr/bin/python
# import numpy as np
# import serial
# import time
# from pyroomba import Roomba
# import threading
# import os , sys
# 
# class Stream(threading.Thread):
#     def __init__(self,r):
#         threading.Thread.__init__(self)
#         self.kill_received = False
#         self.r = r
#         r.setStream(packetIds=[46,47,48,49,50,51,43,44])
# 
#     def kill(self):
#         self.kill_received = True
# 
#     def run(self):
#         while not self.kill_received:
#             b = self.r.receive(1)
#             if ord(b) == 19:
#                 self.r.getDataSTream()
# 
# 
# class Odometrie(threading.Thread):
#     def __init__(self,r):
#         threading.Thread.__init__(self)
#         self.kill_received = False
#         self.r = r
#         # r.last_encodeur = r.getData([43,44])
#         self.r.SENSOR_DATA_LOCK.acquire()
#         self.r.last_encodeur = [self.r.SENSOR_DATA[43], self.r.SENSOR_DATA[44]]
#         self.r.SENSOR_DATA_LOCK.release()
#         # print self.r.last_encodeur
# 
#     def run(self):
#         while not self.kill_received:
#             self.r.positionUpdate()
#             print self.r.position
#             time.sleep(0.025)
# 
# def testMove():
#     threads = []
#     try:
#         r = Roomba('/dev/ttyUSB0',115200)
# 
#         r.fullMode()
# 
#         threads.append(Stream(r))
# 
#         # for t in threads:
#         #     t.start()
#         #     time.sleep()
#         threads[0].start()
#         time.sleep(0.1)
#         threads.append(Odometrie(r))
# 
#         threads[1].start()
# 
#         r.stopTurnTo(3*np.pi)
#         # time.sleep(0.5)
#         # r.stopTurnTo(-np.pi/2)
#         # r.moveToPoint(0.6,1)
#         # r.diffMove(100,100)
#         # time.sleep(5)
#         # r.diffMove(0,0)
#         # for i in range(0,5):
#             # r.uniMove(100, -5)
#             # time.sleep(0.5)
#             # r.stopTurnTo(np.pi/2)
#         # time.sleep()
# 
#         # print r.getData([43])
# 
#         for t in threads:
#             t.kill_received = True
# 
#         print r.SENSOR_DATA
# 
#         r.disconnect()
#     except:
#         for t in threads:
#             t.kill_received = True
# 
#         r.disconnect()
# 
# 
# def test():
#     r = Roomba('/dev/ttyUSB0',115200)
#     r.send([7])
#     # print "mode : {0}".format(r.getMode())
#     # r.fullMode()
#     # print "mode : {0}".format(r.getMode())
# 
#     # r.setSong(1, [61,50,62,50,63,50])
#     # r.playSong(1)
#     # d = r.getData([35,43,44])
# 
#     # r.diffMove(-100,-100)
#     # time.sleep(1.84)
#     # r.diffMove(0,0)
#     # for i in range(0,5):
#     #     r.uniMove(100, 5)
#     #     time.sleep(0.5)
#     #     r.stopTurnTo(np.pi/2)
#         # time.sleep(0.5)
# 
# 
# 
#     # r.stopTurnTo(np.pi)
# 
# 
#     r.disconnect()
# 
# 
#     # r.setStream([46,47,48,49,50,51])
#     #
#     # threading.Timer(1, plotData,args=[r.SENSOR_DATA[46],r.SENSOR_DATA[47],r.SENSOR_DATA[48],r.SENSOR_DATA[49],r.SENSOR_DATA[50],r.SENSOR_DATA[51]])
#     #
#     # while True:
#     #     if ord(r.receive(1)) == 19:
#     #         r.getDataSTream()
# 
# 
# if __name__ == '__main__':
#     testMove()
