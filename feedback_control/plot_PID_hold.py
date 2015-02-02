import os,sys
import pylab
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as lines

############################
# TODO: set values here 
# see figure 1 in homework for definition of variables
set_point = 0.0 
rise_time = 1.0 
peak_time = 2.0
overshoot = 0.2
settling_time = 3.0

# Error bands are at y = (1 +/- delta)
error_band_delta = 0.05

# offset for text
eps = .05
############################


# setup font
font =  {'family' : 'sans-serif',
         'color'  : 'black',
         'weight' : 'normal',
         'size'   : 16,
        }

# step size (constant value)
DT = 0.001

# load data
y1 = pylab.loadtxt("./PID_hold.desired")
y2 = pylab.loadtxt("./PID_hold.state")
t = np.linspace(0,y1[:,0].size*DT,y1[:,0].size)

# setup the figure
fig = plt.figure()

# plot data
plt.plot(t,y1[:,0],'k' ,label='Desired (joint 1)')
plt.plot(t,y2[:,0],'r' ,label='Actual (joint 1)')

# set x limit
axPlot = plt.subplot(111)
axPlot.set_xlim(-.001, 7)

# add titles, labels, and legend
plt.title('Desired vs. actual joint angles for (PID) hold task', fontdict=font)
plt.xlabel('Time', fontdict=font)
plt.ylabel('Joint angle', fontdict=font)
plt.legend(loc=4, shadow=True)

# annotate rise time (interval)
plt.axhspan(set_point-error_band_delta,set_point+error_band_delta, facecolor='0.5', alpha=0.25)
plt.axhline(y=set_point-error_band_delta, color='k',linestyle='--')
plt.axvspan(0.0,rise_time,facecolor='0.5', alpha=0.25)
plt.axvline(x=rise_time, color='k',linestyle='--')
plt.annotate('rise time', xy=(rise_time-.5, set_point-eps))

# annotate peak_time
plt.axvline(x=peak_time, color='k',linestyle='--')
plt.annotate('peak time', xy=(peak_time+.1, set_point-2*eps))

# annotate overshoot
plt.axhline(y=overshoot, xmin=0, xmax=peak_time,color='k',linestyle='--')
plt.annotate('overshoot', xy=(peak_time, overshoot+eps))

# annotate settling time
plt.axvline(x=settling_time,color='k',linestyle='--')
plt.annotate('settling time', xy=(settling_time, set_point+eps))

# draw error band
plt.axhspan(set_point-error_band_delta,set_point+error_band_delta, facecolor='0.5', alpha=0.25)
plt.axhline(y=set_point-error_band_delta, color='k',linestyle='--')
plt.axhline(y=set_point+error_band_delta, color='k',linestyle='--')
#plt.plot([0, y1[:,0].size*DT], [-0.5, -0.5], '--', lw=1.0)
#plt.plot([0, y1[:,0].size*DT], [-1.5, -1.5], '--', lw=1.0)
plt.annotate('error band', xy=(10.1, set_point))

# show the plot
plt.show()

# save the plot
plt.savefig('plot_PID_hold.png')

