#!/usr/bin/env python
import select
import socket
class ConnectionManager(object):


    def __init__(self, connType):
        '''
            connType -- string is eiter tcp or udp
        '''
        self.connType = connType
        self.clientsock = []
        self.clientIPPort = {}


    def buildServer(self, port):
        '''
        builds the server for the particular connection type
        '''
        if self.connType == 'tcp':
            self.server_socket, self.addr = self.tcpConnection('', port)
            self.isServer = True
        else:
            ## else we are udp
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.server_socket.bind(('', port))
            self.isServer = True

    def tcpConnection(self, host, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
        sock.listen(1)
        return sock.accept()

    def addClient(self, ip, port):
        self.isServer = False
        if self.connType == 'tcp':
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((ip, port))
                self.clientsock.append(sock)
                self.clientIPPort[sock] = (ip, port)
            except:
                return False
        else:
            ## else we are udp
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.clientsock.append(sock)
            self.clientIPPort[sock] = (ip, port)
        return True

    def send(self, data, ip=None, port=None):
        '''
            Will send data to the client if you are a tcp server
            Will NOT send data if you are a UDP server
            Will send data to all of the client connections if a client
            Will send data to all udp servers if you are a udp client
        '''
        if self.isServer == True and self.connType == 'tcp':
            self.server_socket.sendall(data)
        elif self.isServer == True and self.connType == 'udp':
            ## THIS IS NOT POSSIBLE IF YOU ARE A UDP SERVER you can't send
            ## make it a client then you can send.
            tempSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print 'Sending %s to %s :%d' %(data, ip, port)
            tempSock.sendto(data, (ip, port))
        elif self.isServer == False and self.connType == 'tcp':
            _,ready_socks,_ = select.select([], self.clientsock, [])
            for sock in ready_socks:
                sock.sendall(data) # This is will not block
                print "sent message:", data
        elif self.isServer == False and self.connType == 'udp':
            for sock in self.clientsock:
                print 'Sending %s to %s :%d' %(data, self.clientIPPort[sock][0], self.clientIPPort[sock][1])
                sock.sendto(data, self.clientIPPort[sock])
        return True

    def recv(self, port=None):
        '''
        blocking recv until one of the server sockets has
        for udp
        returns a list of tupples [(data, ('address', port)),...]
        need port if i was set up as a udp client and i want to recv
        '''
        recvData = []
        ssock = self.clientsock


        if self.isServer == True:
            ssock = [self.server_socket]
        ready_socks,_,_ = select.select(ssock, [], [])
        for sock in ready_socks:
            data, addr = sock.recvfrom(4096) # This is will not block
            if self.connType == 'tcp' and self.isServer == False:
                addr = self.clientIPPort[sock]
            elif self.connType == 'tcp' and self.isServer == True:
                addr = self.addr
            recvData.append((data, addr))
            print "received message: %s from %s:%d" % (data, addr[0], addr[1])
        return recvData
