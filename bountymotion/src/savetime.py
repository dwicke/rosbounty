import socket
import signal
import time
import select
import numpy as np
import ach
import sys
import time
from ctypes import *



UDP_IP = "127.0.0.1"
UDP_PORT = 5005
f = 0.5
totalIncrementer = 0
succIncrementer = 0
T = 1.0 / f
globalTimestampLatest = 0.0


# Ach files
class cloud(Structure):
    _pack_ = 1
    _fields_ = [("data"  , c_double)]

s = ach.Channel('cloud_chan')
state = cloud()




def saveT():
    global totalIncrementer
    global succIncrementer
    global T
    global globalTimestampLatest
    #print "I am in the handler!!!!!!!!!!!"
    # global succIncrementer
    # global totalIncrementer
    # global globalTimestampLatest
    # global T
    t = time.time()
    dt = t - globalTimestampLatest
    print " time = %f globalTSL = %f dt = %f succInc = %f totalInc = %f" % (t, globalTimestampLatest, dt, succIncrementer, totalIncrementer)
    if dt < T:
        succIncrementer += 1
    totalIncrementer += 1

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.setblocking(0)
sock.settimeout(T)
sock.bind((UDP_IP, UDP_PORT))





while True:
    tick = time.time()
    [statuss, framesizes] = s.get(state, wait=False, last=True)

    tock = time.time()
    dt = T - (tock - tick)
    if dt > 0.0:
        globalTimestampLatest = state.data
        print "state = ",state.data," tick = ", tick
#        saveT()
    time.sleep(dt)
