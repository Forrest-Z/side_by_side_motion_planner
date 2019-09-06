import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import scipy as sy
import numpy as np
import matplotlib.pyplot as plt
import math
import sys, getopt



#print 'Number of arguments:', len(sys.argv), 'arguments.'
#print 'Argument List:', str(sys.argv)

#filename = "map.txt"
filename = sys.argv[1]
dat = [] 
total = 0
xdata=[]
ydata=[]
dx=0
dy=0

#xc=5
#yc=7

with open( filename ) as f:
    for line in f:
    	total+=1
    	ray = line.split()
        print ray[0], ray[1], ray[2], ray[3], ray[4]

        
