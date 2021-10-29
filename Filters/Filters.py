#Darpan Theng
import random
import matplotlib.pyplot as plt
y=[]
n=[] #FIR
m=[] #iir
o=[] #1dkalman
a=0
for i in range(100):
    y.append(random.randint(1,101))
r0 = 0.09
k0 = 0
r=50  #noise considered
h=1.0 #output gain if needed but gives lower value
q=1   #initial values
p=0   #initial values
p0=1
u_h=y[0] #initial value ,stores filtered values FIR
mu_h=y[0] #iir
z0= y[0]  #1dkalman
k=0.1317  #kalman gain (changes every iteration) can be whatever u want
for i in range(len(y)-1):
    print(k)
    k= (p*h)/(h*p*h+r)              #FIR
    u_h=u_h + k*(y[i]-(h*u_h))
    p=(1-k*h)*p + q
    print(k)

    #mu_h= 0.9*mu_h+ 0.1*y[i]       #iir

    k0= p0/(p0+r0)                 #1dkalman
    z0=z0+ k0*(y[i]-z0)
    p0=(1-k0)*p0


    o.append(z0)
    #m.append(mu_h)
    #n.append(u_h)
    #m.append(0.6*y[i+1]+0.4*y[i]) #the weights can be made 0.2 and 0.8 but best data is obtained by 0.4 and 0.6

plt.plot(y,label='Data') #red origi
plt.plot(o,label='Kalman1d') #blue sasta
plt.plot(n, label='FIR') #orange FIR
plt.plot(m,label='IIR') #green iir

leg= plt.legend()
plt.show()