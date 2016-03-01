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
f = 5.0
totalIncrementer = 0
succIncrementer = 0
T = 1.0 / f
globalTimestampLatest = 0.0


# Ach files
class cloud(Structure):
    _pack_ = 1
    _fields_ = [("data"  , c_double),
                ("time"  , c_double)]

s = ach.Channel('cloud_chan')
state = cloud()




thefile = open( 'cloudlog.data', "w" )



for nn in range(1,15):
  succIncrementer = 1
  totalIncrementer = 1
  nnn = 0
  doLoop = True
  fn = f*nn
  T = 1.0/fn
  while doLoop:
    tick = time.time()
    [statuss, framesizes] = s.get(state, wait=False, last=True)

    tock = time.time()
    dt = T - (tock - tick)
    check_t = tick - state.data
    print check_t
    if check_t < T:
        #print "state = ",state.data," tick = ", tick, " dt = ", dt
        succIncrementer += 1
    totalIncrementer += 1
    if( nnn < 100 ):
       nnn = nnn + 1
    else:
       doLoop = False
    if(dt < T):
      time.sleep(np.abs(dt))
  savedPercent = float(succIncrementer)/float(totalIncrementer)
  thefile.write("%f,%f\n" % (1.0/T, savedPercent))



thefile.close()
