#/usr/bin/python

from scipy.interpolate import interp1d,UnivariateSpline
from scipy.ndimage import gaussian_filter1d
from scipy.misc import derivative
import numpy as np
import datetime
import matplotlib.pyplot as plt


M1=[s.strip().split(",") for s in open("m1_raw.csv").readlines() if s[0]!='%']
M2=[s.strip().split(",") for s in open("m2_raw.csv").readlines() if s[0]!='%']
L=[s.strip().split(",") for s in open("leica_position.csv").readlines() if s[0]!='%']

def processm(s):
    return [float(s[2])/1e9, float(s[4]),float(s[6])]


def processlp1(s):
    s9=s[9].split(".")
    s100=int(s9[1])*10000
    t = datetime.datetime.strptime("%s %s.%06d"%(s[8],s9[0],s100), '%d.%m.%Y %H:%M:%S.%f')
    return [float(s[2])/1e9, float(s[5]),float(s[6]),float(s[7]),t.timestamp()]

def processlp2(s):
    return [float(s[2])/1e9, float(s[4]),float(s[5]),float(s[6])]

M1=np.array([processm(s) for s in M1])
M2=np.array([processm(s) for s in M2])
L=np.array([processlp2(s) for s in L])

# sigma_m=0.1;
# sigma_l=1.0;
# if (sigma_m>=0) or (sigma_l>=0) :
#     M1[:,2]=gaussian_filter1d(M1[:,2],sigma_m,0)
#     M2[:,2]=gaussian_filter1d(M2[:,2],sigma_m,0)
#     L[:,1]=gaussian_filter1d(L[:,1],sigma_l,0)
#     L[:,2]=gaussian_filter1d(L[:,2],sigma_l,0)
#     L[:,3]=gaussian_filter1d(L[:,3],sigma_l,0)
M1i=interp1d(M1[:,0],M1[:,2], bounds_error=False,kind='linear')
M2i=interp1d(M2[:,0],M2[:,2], bounds_error=False,kind='linear')
Lxi=interp1d(L[:,0],L[:,1], bounds_error=False,  kind='linear')
Lyi=interp1d(L[:,0],L[:,2], bounds_error=False,  kind='linear')
Lzi=interp1d(L[:,0],L[:,3], bounds_error=False,  kind='linear')



dt=0.01
Mtmin=min(np.min(M1[:,0]),np.min(M2[:,0]))
Mtmax=max(np.max(M1[:,0]),np.max(M2[:,0]))
num=int(np.ceil((Mtmax-Mtmin)/dt))
Mt=np.linspace(Mtmin,Mtmax,num)
Ltmin=np.min(L[:,0])
Ltmax=np.max(L[:,0])
num=int(np.ceil((Ltmax-Ltmin)/dt))
Lt=np.linspace(Ltmin,Ltmax,num)
Vxi=gaussian_filter1d(derivative(Lxi,Lt,1e-2),3.0)
Vyi=gaussian_filter1d(derivative(Lyi,Lt,1e-2),3.0)
Vzi=gaussian_filter1d(derivative(Lzi,Lt,1e-2),3.0)
T0=min(Mt[0],Lt[0])
Mn=np.abs(np.sum(np.array([M1i(Mt),M2i(Mt)]),axis=0).transpose()/2)
Vn=np.sqrt(np.sum(np.array([Vxi**2, Vyi**2, Vzi**2]),axis=0)).transpose()
Mn[np.nonzero(np.isnan(Mn))] = 0
Vn[np.nonzero(np.isnan(Vn))] = 0
#Mn=np.array([Mn])
#print(np.squeeze(Mn).shape)
#print(Vn.shape)
# Remove NaN
Vn[0]=0
Vn[-1]=0


C=np.correlate(np.squeeze(Mn-np.mean(Mn)),np.squeeze(Vn-np.mean(Vn)),'same')
# plt.plot(range(C.shape[0]),C,'-')
# plt.show()
dT = (np.argmax(C) - Mn.shape[0]/2)*dt
print("dT: %f" % dT)
O=0.02
plt.plot(Mt-T0,O+Mn,'-');
plt.plot(Lt-Lt[0],O+Vn,'.');
plt.text(0,0.3,"Raw")
plt.plot(Mt-T0,-Mn-O,'-');
plt.plot(Lt+dT-T0,-Vn-O,'.');
plt.text(0,-0.3,"Sync")
plt.grid()
plt.show()




