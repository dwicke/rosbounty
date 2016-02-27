#!/usr/bin/env python


class DataCollector(object):

    def __init__(self):
        self.collectedData = {}

    def addPoint(self, setName, point):
        if setName not in self.collectedData:
            self.collectedData[setName] = []
        self.collectedData[setName].append(point)

    def writeData(self):
        '''
        writes all data
        '''
        for k in self.collectedData.keys():
            print 'writing file ' + "/tmp/" + k + ".dat"
            thefile = open( "/tmp/" + k + ".dat", "w" )
            for item in self.collectedData[k]:

                thefile.write("%d,%f\n" % (item[0], item[1]))
            thefile.close()
    def deleteSet(setName):
        del self.collectedData[setName]
