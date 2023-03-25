import time
from smbus import SMBus
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import matplotlib.pyplot as plt
import matplotlib.animation as animation

i2c = I2C(1)

IMU = LSM6DSOX(i2c)

samples = 20
count = 0;
acc_X = []
x = []
acc_Y = []
acc_Z = []

gyro_X = []
gyro_Y = []
gyro_Z = []

fig = plt.figure(1)
ax = fig.subplots(3,1)

def get_acc(i):
	global x, acc_X, acc_Y, acc_Z
	t = time.localtime()
	curr_time = time.strftime("%S",t)
	x.append(curr_time)
	acc_X.append(round(IMU.acceleration[0],2))
	acc_Y.append(round(IMU.acceleration[1],2))
	acc_Z.append(round(IMU.acceleration[2],2))
	
	x = x[-samples:]
	acc_X = acc_X[-samples:]
	acc_Y = acc_Y[-samples:]
	acc_Z = acc_Z[-samples:]
	
	
	ax[0].clear()
	ax[1].clear()
	ax[2].clear()
	ax[0].plot(x, acc_X)
	ax[1].plot(x, acc_Y)
	ax[2].plot(x, acc_Z)
	
def get_gyro(i):
	global x, gyro_X, gyro_Y, gyro_Z
	t = time.localtime()
	curr_time = time.strftime("%S",t)
	x.append(curr_time)
	gyro_X.append(round(IMU.gyro[0],2))
	gyro_Y.append(round(IMU.gyro[1],2))
	gyro_Z.append(round(IMU.gyro[2],2))
	
	x = x[-samples:]
	gyro_X = gyro_X[-samples:]
	gyro_Y = gyro_Y[-samples:]
	gyro_Z = gyro_Z[-samples:]
	
	
	ax[0].clear()
	ax[1].clear()
	ax[2].clear()
	ax[0].plot(x, gyro_X)
	ax[1].plot(x, gyro_Y)
	ax[2].plot(x, gyro_Z)
	

while True:
	
	ani = animation.FuncAnimation(fig, get_gyro, interval = 10)
	plt.show()
	
	
	#plt.close()
    


