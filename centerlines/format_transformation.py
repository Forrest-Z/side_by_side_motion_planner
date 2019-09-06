import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import scipy as sy
import numpy as np
import matplotlib.pyplot as plt
import math
import sys, getopt


DISP_X  = 0.0 

DISP_Y  = 0.0
DISP_TH_deg = 4

DISP_TH = DISP_TH_deg * math.pi/180


#def main(argv):

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
#        print ray[0], ray[1], ray[2], ray[3], ray[4]    	
#        print ray[0], dx, ray[2], dy, ray[4]
        #xdata.append(float(ray[1]))
        #ydata.append(float(ray[3]))
#        dx = float(ray[xc]) *np.cos( DISP_TH ) - float(ray[yc])*np.sin( DISP_TH )  + DISP_X 
#        dy = float(ray[xc]) *np.sin( DISP_TH ) + float(ray[yc])*np.cos( DISP_TH )  + DISP_Y 
        print ray[1], ray[2], "0", ray[5], "0" 



#x_new = np.array(xdata)
#y_new = np.array(ydata)
