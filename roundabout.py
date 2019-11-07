import matplotlib.pyplot as plt
import numpy as np
from math import pi
from matplotlib.animation import FuncAnimation

#Simulation
dt = 1/6 # step size of the environment
datDraw = 0.001 #step size for ploting

# Roundabout size and parameters
r = [27.5, 34.5]  # radius of the inner and outer lane
vMax = [40/3.6, 30/3.6]   # maximum speed of the inner and outer lane
width = r[1] - r[0]

# Car parameters
a = 1.3
b = 1.5
L = a + b

aMax = 2.778
bMax = 7

# Traffic
carNum = 50
spd = 15/3.6
spdVar = 4

#IDM parameters
delta = 3  # acceleration index .1~5 is ok
safeGap = 2
tao = 0.7  # reaction time for emergent brake
maxCF = 9999  # if the gap between ego car and front car is more than maxCF, then ego car will adopt max accelaration

# s:state matrix,each columu is a state vector for a car.
# vector is [theta, r, v, CFmode, targetExist]
# a: = [ax, ay]
# CF mode: 1:CF  2: DCF 3:CCF 4:ACF 5:RCF


class Roundabout:
    def __init__(self):

        plt.close()  # clf() # 清图 cla() # 清坐标轴 close() # 关窗口
        self.fig = plt.figure(figsize=(12, 12))
        self.ax = self.fig.add_subplot(1, 1, 1)
        plt.ion()

    def drawCar(self, s):
        sX = np.cos(s[0]) * s[1]
        sY = np.sin(s[0]) * s[1]
        self.ax.scatter(sX, sY, c='b', marker='s', s=100)  # 散点图


    def drawRound(self):
        for rr in [r[0] - width/2, r[1] - width/2, r[1] + width/2]:
            x = np.arange(-rr, rr, 0.001)
            y = np.sqrt(rr**2 - x**2)
            self.ax.plot(x, -y, c='black')
            self.ax.plot(x, y, c='black')


    def drawFollow(self, s, ard):
        for i in range(carNum):
            j = ard[0, i]
            sX = np.cos(s[0]) * s[1]
            sY = np.sin(s[0]) * s[1]
            self.ax.arrow(sX[i], sY[i], sX[j] - sX[i], sY[j] - sY[i],
                           length_includes_head=True,  # 增加的长度包含箭头部分
                           head_width=1, head_length=2, fc='r', ec='gray')


    def drawdata(self, s, data):
        for i in range(carNum):
            self.ax.annotate(s=str(data[i]), xy=(np.cos(s[0]) * s[1], np.sin(s[0]) * s[1]))


class Cars:
    def __init__(self):
        global carNum
        self.s = np.zeros([5, carNum])
        self.s[0] = np.random.rand(carNum) * 2 * pi
        self.s[1] = r[0] + (r[1] - r[0]) * np.random.randint(0, 2, carNum)
        self.s[2] = abs(np.random.normal(spd, spdVar, size=(1, carNum)))
        self.s[-1] = np.random.randint(1, 5, carNum)

        overlap = []
        for i in range(carNum):
            for j in range(i + 1, carNum):
                if self.s[1, i] == self.s[1, j] and abs(self.s[0, i] - self.s[0, j]) * self.s[1, i] < L + safeGap:
                    overlap.append(i)
                    break
        self.s = np.delete(self.s, overlap, axis=1)
        self.s = self.s.T[np.lexsort(self.s[::-1, :])].T  # sort the s by theta
        carNum -= len(overlap)
        print('Initialized with %d car' % (carNum))

    def dynamic(self, a):
        self.s[2] += a*dt
        self.s[0] += self.s[2]/self.s[1]*dt
        self.s[0] = np.mod(self.s[0], 2*pi)
        self.s = self.s.T[np.lexsort(self.s[::-1, :])].T # sort the s by theta


    def idm(self):
        a = np.zeros(carNum)
        around = self.around()
        for i in range(carNum):
            j = around[0, i]
            if self.s[0, j] < self.s[0, i]:
                sn = (self.s[0, j] - self.s[0, i] + 2*pi) * self.s[1, i] - L
            else:
                sn = (self.s[0, j]-self.s[0, i])*self.s[1, i] - L


            vn = self.s[2, i]
            vn1 = self.s[2, j]
            vMaxn = vMax[self.s[1, i] == r[1]]
            EGap = safeGap + max(0, tao*vn+vn*(vn-vn1)/(2*np.sqrt(aMax*bMax)))
            a[i] = aMax*(1 - (vn/vMaxn)**delta-(EGap/sn)**2)
            a[i] = min(aMax, max(-bMax, a[i]))
            #a[i] += np.random.normal(0, 0.7)
            rounda.ax.annotate(s=str(round(vn, 1)), xy=(np.cos(self.s[0, i]) * self.s[1, i]+2, np.sin(self.s[0, i]) * self.s[1, i]))
            rounda.ax.annotate(s=str(round(a[i], 1)),
                              xy=(np.cos(self.s[0, i]) * self.s[1, i], np.sin(self.s[0, i]) * self.s[1, i]+ 2))
            rounda.ax.annotate(s=str(round(EGap, 1)),
                               xy=(np.cos(self.s[0, i]) * self.s[1, i], np.sin(self.s[0, i]) * self.s[1, i] - 2))

        return a

    def around(self):
        ard = np.zeros([2, carNum], dtype=int)
        for i in range(carNum):
            for j in range(carNum):
                if self.s[1, (i + j + 1) % carNum] == self.s[1, i]:  # find the front car in the same lane
                    ard[0, i] = (i + j + 1) % carNum
                    break
        return ard


rounda = Roundabout()
cars = Cars()
for i in range(1000):
    rounda.ax.cla()
    a = cars.idm()
    cars.dynamic(a)

    rounda.drawRound()
    rounda.drawCar(cars.s)
    rounda.drawFollow(cars.s, cars.around())
    plt.pause(datDraw)



