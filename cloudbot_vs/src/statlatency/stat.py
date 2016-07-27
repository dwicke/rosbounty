#!/bin/bash
from numpy import *
import os  

f = open("stats.csv",'w')

for fn in os.listdir('.'):
     if os.path.isfile(fn) and fn != "stat.py" and fn != "stats.csv":
        print (fn)
	vals = fromfile(fn,sep='\n')
	print("std = {}".format(vals[0]))
	mystd = std(vals)
	mymean = mean(vals)
	f.write("{},{},{}\n".format(fn, mymean, mystd))
f.close()	
