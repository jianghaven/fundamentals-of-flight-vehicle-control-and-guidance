# -*- coding: utf-8 -*-
"""
Created on Fri Dec 1 00:36:49 2017

@author: JiangHaven 
"""

import numpy as np
import cvxpy as cvx
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig1 = plt.figure(1)
fig2 = plt.figure(2)
ax = Axes3D(fig1)

x0  = 0; y0  = 0;  z0  = 0
xf  = 200;yf  = 200;zf  = 200
Vconst = 20;dotVmax = 11;Vu1 = 10
N = 100;tau = 0.22;S = 6
V0 = 0;Vf = 40
Xobs = 100;Yobs = 110;Zobs = 80;R = 55 
T = np.linspace(1,N,N-1)*tau
Xk= np.zeros([S,N])
Yk= np.zeros([S,N])
Zk= np.zeros([S,N])
Xk_temp= np.zeros([N])
Yk_temp= np.zeros([N])
Zk_temp= np.zeros([N])
Xk[0,:] = np.linspace(x0,xf,N)
Yk[0,:] = np.linspace(y0,yf,N)
Zk[0,:] = np.linspace(z0,zf,N)

for s in range(S-1):
     Xk_temp = Xk[s,:];Yk_temp = Yk[s,:];Zk_temp= Zk[s,:]#
     f=[np.linalg.norm([Xk_temp[i] - Xobs,Yk_temp[i] - Yobs,Zk_temp[i] - Zobs])**2
     -R**2 for i in range(len(Zk_temp))]
     f=np.array(f)
     #define state and control variables
     x = cvx.Variable(1,N)
     y = cvx.Variable(1,N)
     z = cvx.Variable(1,N)
     u1= cvx.Variable(1,N-1)
     u2= cvx.Variable(1,N-1)
     u3= cvx.Variable(1,N-1)
     #state constrants
     F  =  [x[0,0] == x0]
     F  += [x[0,N-1] == xf]
     F  += [y[0,0] == y0]
     F  += [y[0,N-1] == yf]
     F  += [z[0,0] == z0]
     F  += [z[0,N-1] == zf]
     F  += [x0 <= x,x <= xf]
     F  += [y0 <= y,y <= yf]
     F  += [z0 <= z,z <= zf]
     #control terminal constrants
     F  += [u1[0,N-2]==Vconst]
     F  += [u2[0,N-2]==0]
     F  += [u3[0,N-2]==0]
     F  += [0 <= u1,u1 <= Vconst]
     F  += [0 <= u2,u2 <= Vconst]
     F  += [0 <= u3,u3 <= Vconst]
     #control convex constrants
     F  += [u1**2+u2**2+u3**2 <= Vconst**2]
     # Dynamic constraints    
     F  += [x[0,i+1] == x[0,i] + tau*u1[0,i] for i in range(N-1)]
     F  += [y[0,i+1] == y[0,i] + tau*u2[0,i] for i in range(N-1)]
     F  += [z[0,i+1] == z[0,i] + tau*u3[0,i] for i in range(N-1)]
     #Sequence obstacle constraints
     F  += [(f[ii]+2*(Xk_temp[ii]-Xobs)*(x[0,ii]-Xk_temp[ii])+2*(Yk_temp[ii]-
     Yobs)*(y[0,ii]-Yk_temp[ii])+2*(Zk_temp[ii]-Zobs)*(z[0,ii]-Zk_temp[ii]))>=0
            for ii in range(N)]  
     obj = cvx.Minimize(sum(x)+sum(y)+sum(z))
     prob = cvx.Problem(obj, F)
     prob.solve(solver='SCS')#CVXOPT SCS 默认的是ECOS
     result = prob.solve()
     if x.value is None:
         X=np.zeros([1,N])
         Y=X
         Z=X
         print('infeasibale')
     else:
         X=x.value.A.flatten();Y=y.value.A.flatten();Z=z.value.A.flatten()
         U1=u1.value.A.flatten();U2=u2.value.A.flatten();U3=u3.value.A.flatten()
         print('feasibale')
     Xk[s+1,:] = X; Yk[s+1,:] = Y; Zk[s+1,:] = Z
     ax.plot(X,Y,Z,'g-')
     plt.plot(T,U1,'g-',T,U2,'b.-',T,U3,'r--',T,np.sqrt(U1**2+U2**2+U3**2),'k-')
     print("Sequence optimization times is",s)