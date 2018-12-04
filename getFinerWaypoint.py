# -*- coding: utf-8 -*-
"""
Created on Wed Nov 28 13:26:45 2018

@author: taira
"""

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.interpolate import splprep,splev


# reead data
filename = './data/highway_map.csv'
data = np.genfromtxt(filename, dtype='double', delimiter=' ')

#%%
# get interpolation 
x = data[:, 0]
x = np.hstack((x, x[0]))
#x = np.hstack((x, x[0:100]))
y = data[:, 1]
y = np.hstack((y, y[0]))
#tck, u = splprep([x, y], s=0, k=2 , per=True)
tck, u = splprep([x, y], s=0, k=3 , per=True)
step = 0.0001
unew = np.arange(0, 1.0+step, step)
out = splev(unew, tck)
new_x = out[0]
new_y = out[1]


# plot interpolation
plt.figure()
plt.scatter(x, y, label='orig.')
plt.scatter(new_x, new_y, label='interp.', s = 1)
#plt.xlim([2000, 2200])
#plt.ylim([2950,3050])
plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.show()


plt.figure()
plt.plot(x, y, 'o', label='orig.')
plt.plot(new_x, new_y, '.', label='interp.')
#plt.plot([785, 909, 854],[1135,1128, 1134], 'r*')
#plt.xlim([500, 1000])
#plt.ylim([1100,1200])
#plt.xlim([750, 820])
#plt.ylim([1134,1138])
#plt.xlim([780, 786])
plt.xlim([2150,2250])
plt.ylim([2960, 3000])
plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.show()

#%%

# get new s
s = data[:,2]

p = np.polyfit(u[:-1], s, 1)
#p = np.polyfit(u, s, 1)
f = np.poly1d(p)
new_s = f(unew)

plt.figure()
#plt.plot(u, s, 'o')
plt.plot(u[:-1], s, 'o')
plt.plot(unew, new_s,'.')
plt.xlabel('u')
plt.ylabel('s')
plt.show()

#%%
# check if the calculation of d coordinate is right
dx_test = []
dy_test = []
n = len(x)
for i in range(n):
    theta = np.arctan2(-(y[(i+1)%n]-y[(i-1)%n]), x[(i+1)%n]-x[(i-1)%n])
    dx_test.append(-np.sin(theta))
    dy_test.append(-np.cos(theta))

dx = data[:, 3]
dy = data[:, 4]


plt.figure()
#plt.plot(dx, dx_test, '.')
#plt.plot(dy, dy_test, '.')
plt.plot(dx, dx_test[:-1], '.')
plt.plot(dy, dy_test[:-1], '.')
plt.xlim([-1,1])
plt.ylim([-1,1])
plt.legend()
plt.xlabel('original')
plt.ylabel('calculated')
plt.show()

"""
plt.figure()
plt.plot(x, y, '.')
plt.plot(x+dx, y+dy, '.')
#plt.xlim([2045,2050])
plt.xlim([1600,1800])
plt.ylim([1140,1160])
plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.show()
"""


#%%
# compute d coordinate

dx_new = []
dy_new = []

n = len(new_x)
for i in range(n):
    theta = np.arctan2(-(new_y[(i+1)%n]-new_y[(i-1)%n]), new_x[(i+1)%n]-new_x[(i-1)%n])
    dx_new.append(-np.sin(theta))
    dy_new.append(-np.cos(theta))

plt.figure()
plt.plot(s, dx, 'o')
plt.plot(new_s, dx_new, '.')
plt.legend()
plt.xlabel('s')
plt.ylabel('dx')
plt.show()

plt.figure()
plt.plot(s, dy, 'o')
plt.plot(new_s, dy_new, '.')
plt.legend()
plt.xlabel('s')
plt.ylabel('dy')
plt.show()

#%%
#new_transform = np.transpose(np.vstack((new_x,new_y, new_s, dx_new,dy_new)))
#np.savetxt( './data/precise_highway_map.csv', new_transform, fmt='%f', delimiter=' ')
