#/usr/bin/python

from scipy.interpolate import interp1d,UnivariateSpline
from scipy.ndimage import gaussian_filter1d
from scipy.misc import derivative
import numpy as np
import datetime
import matplotlib.pyplot as plt
import time


O=[s.strip().split(",") for s in open("../data/odom.csv").readlines() if s[0]!='%']
L=[s.strip().split(",") for s in open("../data/leica_points.csv").readlines() if s[0]!='%']

def processm(s):
    return [float(s[2])/1e9, float(s[4]),float(s[6])]

def processo(s):
    return [float(s[2])/1e9, float(s[5]),float(s[6]), float(s[48]),float(s[49])]


def processlp1(s):
    s9=s[9].split(".")
    s100=int(s9[1])*10000
    t = datetime.datetime.strptime("%s %s.%06d"%(s[8],s9[0],s100), '%d.%m.%Y %H:%M:%S.%f')
    return [float(s[2])/1e9, float(s[5]),float(s[6]),float(s[7]),t.timestamp()]

def processlp2(s):
    return [float(s[2])/1e9, float(s[4]),float(s[5]),float(s[6])]

O=np.array([processo(s) for s in O])
L=np.array([processlp1(s) for s in L])

Vi=interp1d(O[:,0],O[:,3], bounds_error=False,kind='linear')

lt_column = 4

Lxi=interp1d(L[:,lt_column],L[:,1], bounds_error=False,  kind='linear')
Lyi=interp1d(L[:,lt_column],L[:,2], bounds_error=False,  kind='linear')
Lzi=interp1d(L[:,lt_column],L[:,3], bounds_error=False,  kind='linear')



dt=0.01
Mtmin=np.min(O[:,0])
Mtmax=np.max(O[:,0])
num=int(np.ceil((Mtmax-Mtmin)/dt))
Mt=np.linspace(Mtmin,Mtmax,num)
Ltmin=np.min(L[:,lt_column])
Ltmax=np.max(L[:,lt_column])
num=int(np.ceil((Ltmax-Ltmin)/dt))
Lt=np.linspace(Ltmin,Ltmax,num)
Vxi=gaussian_filter1d(derivative(Lxi,Lt,1e-2),5.0)
Vyi=gaussian_filter1d(derivative(Lyi,Lt,1e-2),5.0)
Vzi=gaussian_filter1d(derivative(Lzi,Lt,1e-2),5.0)
T0=min(Mt[0],Lt[0])
Mn=np.abs(Vi(Mt)).transpose()
# Vn=np.sqrt(np.sum(np.array([Vxi**2, Vyi**2, Vzi**2]),axis=0)).transpose()
Vn=np.sqrt(np.sum(np.array([Vxi**2, Vyi**2]),axis=0)).transpose()
Mn[np.nonzero(np.isnan(Mn))] = 0
Vn[np.nonzero(np.isnan(Vn))] = 0
#Mn=np.array([Mn])
#print(np.squeeze(Mn).shape)
# print(Vn.shape)
# print(Mn.shape)
# Remove NaN
Vn[0]=0
Vn[-1]=0

Mc=np.array([0.0]*(len(Vn)//5) + list(Mn) + [0.0]*(len(Vn)//5))
Vc=np.array([0.0]*(len(Mn)//5) + list(Vn) + [0.0]*(len(Mn)//5))

Mnn = (Mn - np.mean(Mn))/np.std(Mn)
Vnn = (Vn - np.mean(Vn))/np.std(Vn)
Mcn = (Mc - np.mean(Mc))/np.std(Mc)
Vcn = (Vc - np.mean(Vc))/np.std(Vc)

i0=-1
# t=time.time()
# S0=[]
# for i in range(len(Vc)-len(Mn)):
#     s=Mn - Vc[i:i+len(Mn)]
#     s=np.sum(s*s)
#     S0.append(s)
# i0 = np.argmin(S0)
# print("raw SSD: %f: %d " % (time.time()-t, i0))

t=time.time()
S = np.sum(Mn**2) - 2*np.correlate(Vc, Mn) + np.correlate(Vc**2, np.ones_like(Mn))
#S = - 2*np.correlate(Vc, Mn) + np.correlate(Vc**2, np.ones_like(Mn))
i = np.argmin(S)
print("corr SSD: %f: %d vs %d" % (time.time()-t,i,i0))

dT = -(i - len(Mn)//5) * dt

plt.plot(range(len(S)),S,'-')
#plt.plot(range(len(S0)),S0,'-')
plt.show()
print("dT: %f -> %f: %f" % (dT,dT-Lt[0]+T0,np.min(S)))
print("M0: %f" % T0)
print("L0: %f" % Lt[0])

# C=np.correlate(np.squeeze(Mn-np.mean(Mn)),np.squeeze(Vn-np.mean(Vn)),'same')
# C=np.correlate(np.squeeze(Mcn),np.squeeze(Vcn),'same')
# plt.plot(range(C.shape[0]),C,'-')
# plt.show()
# dT = (np.argmax(C) - Mcn.shape[0]/2)*dt
# print("dT: %f" % dT)
O=0.02
plt.plot(Lt-Lt[0],O+Vn,'-');
plt.plot(Mt-Mt[0],O+Mn,'-');
plt.text(0,0.3,"Raw")
plt.plot(Lt+dT-Lt[0],-Vn-O,'-');
plt.plot(Mt-Mt[0],-Mn-O,'-');
plt.text(0,-0.3,"Sync")
plt.grid()
plt.show()




