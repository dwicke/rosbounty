import socket
import signal
import time
import select
import numpy as np

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
f = 0.5
totalIncrementer = 0
succIncrementer = 0
T = 1.0 / f
globalTimestampLatest = 0.0

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
    sock.settimeout(T)
    tick = time.time()

    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    ddt = (tick - float(data))
    while(T < ddt):
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        ddt = np.abs(tick - float(data))


    tock = time.time()
    dt = T - (tock - tick)
    if dt > 0.0:
        globalTimestampLatest = float(data)
        saveT()
        time.sleep(dt)
