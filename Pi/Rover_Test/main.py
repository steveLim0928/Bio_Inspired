from smbus import SMBus
import time
from gpiozero import Button
import RPi.GPIO as GPIO
import numpy as np
import LSM6DSO

##### IMU
i2cbus = SMBus(1)
LSM6DSOAddr = 0x6B
INT1 = Button(26, pull_up = False)  
INT2 = Button(19, pull_up = False)  
acc = (0.0, 0.0, 0.0)
gyro = (0.0, 0.0, 0.0)

# Interupt callback function
def LSM6DSO_readAcc():
	global acc
	x = LSM6DSO.readAccX(LSM6DSOAddr, i2cbus)
	y = LSM6DSO.readAccY(LSM6DSOAddr, i2cbus)
	z = LSM6DSO.readAccZ(LSM6DSOAddr, i2cbus)
	acc = (x, y, z)
	
def LSM6DSO_readGyro():
	global gyro
	x = LSM6DSO.readGyroX(LSM6DSOAddr, i2cbus)
	y = LSM6DSO.readGyroY(LSM6DSOAddr, i2cbus)
	z = LSM6DSO.readGyroZ(LSM6DSOAddr, i2cbus)
	gyro = (x, y, z)
	
INT1.when_pressed = LSM6DSO_readAcc
INT2.when_pressed = LSM6DSO_readGyro
	
# begin calling IMU
error = LSM6DSO.begin(LSM6DSOAddr, i2cbus)
if (~error):
	print("IMU Found")
else:
	print("IMU Not Found")

LSM6DSO.intialise(LSM6DSOAddr, i2cbus)
print("IMU initialised")

##### IMU end

while True:
	
	
	print("Acc: %.4f, %.4f, %.4f" % (acc))
	
	
	print("Gyro: %.4f, %.4f, %.4f" % (gyro))
	print("")
	time.sleep(0.01)
	#continue
