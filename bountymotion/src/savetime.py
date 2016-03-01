import socket
import signal


UDP_IP = "127.0.0.1"
UDP_PORT = 5005
f = 10.0
totalIncrementer = 0
succIncrementer = 0
T = 1.0 / f
globalTimestampLatest = 0.0

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
signal.signal(signal.SIGALRM, handler)
signal.setitimer(signal.ITIMER_REAL, 0.5, T)


def handler(signum, frame):
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


while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print data
