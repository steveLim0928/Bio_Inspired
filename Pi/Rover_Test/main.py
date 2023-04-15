from smbus import SMBus
import time
#import curses
from gpiozero import Button, Servo, Motor
import numpy as np
import LSM6DSO
import pygame

####################### SETUP ####################### 

##### IMU
i2cbus = SMBus(1)
LSM6DSOAddr = 0x6B
INT1 = Button(13, pull_up = False)  
INT2 = Button(6, pull_up = False)  
acc = (0.0, 0.0, 0.0)
gyro = (0.0, 0.0, 0.0)

cummulativeAngle = 0.0

gyroTimeNew = 0.0
gyroTimeOld = 0.0

##### Keyboard Input
#screen = curses.initscr()
#curses.noecho() 
#curses.cbreak()
#screen.keypad(True)
pygame.init()
window = pygame.display.set_mode((300, 300))
#clock = pygame.time.Clock()

#rect = pygame.Rect(0, 0, 20, 20)
#rect.center = window.get_rect().center
#vel = 5

##### Motors
motorSpeed = 0.8
rightMotor = Motor(26,19)
leftMotor = Motor(21,20)

##### Charging Plates
leftServo = Servo (12, min_pulse_width = 0.0005, max_pulse_width = 0.0025)
rightServo = Servo (16, min_pulse_width = 0.0005, max_pulse_width = 0.0025)

####################### FUNCTIONS ####################### 

##### IMU
def LSM6DSO_readAcc():
	global acc
	x = LSM6DSO.readAccX(LSM6DSOAddr, i2cbus)
	y = LSM6DSO.readAccY(LSM6DSOAddr, i2cbus)
	z = LSM6DSO.readAccZ(LSM6DSOAddr, i2cbus)
	acc = (x, y, z)
	
def LSM6DSO_readGyro():
	global gyro, gyroTimeNew, gyroTimeOld
	gyroTimer = time.time()
	x = LSM6DSO.readGyroX(LSM6DSOAddr, i2cbus)
	y = LSM6DSO.readGyroY(LSM6DSOAddr, i2cbus)
	z = LSM6DSO.readGyroZ(LSM6DSOAddr, i2cbus)
	gyroTimeOld = gyroTimeNew
	gyroTimeNew = time.time()
	gyro = (x, y, z)
	return (x, y, z)
	
def IMU_Gyro_Cal():
	gyroCal = []
	i = 0
	while i < 100:
		gyroCal.append(LSM6DSO_readGyro())
		i += 1
	x = 0.0;
	y = 0.0;
	z = 0.0;
	for j in range(len(gyroCal)):
		temp = gyroCal[j]
		x += temp[0]
		y += temp[1]
		z += temp[2]
	
	return (x/100, y/100, z/100)
	
INT1.when_pressed = LSM6DSO_readAcc
INT2.when_pressed = LSM6DSO_readGyro
	
####################### MAIN ####################### 	
	
# SETUP
error = LSM6DSO.begin(LSM6DSOAddr, i2cbus)
if (~error):
	print("IMU Found")
else:
	print("IMU Not Found")

LSM6DSO.intialise(LSM6DSOAddr, i2cbus)
print("IMU initialised")

##### IMU end

print("Calibrate Gyro")
time.sleep(5)
gyroCal = IMU_Gyro_Cal();
print("Gyro calibrated: %.04f, %.04f, %.04f" % gyroCal)

run = True
while run:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
        if event.type == pygame.KEYDOWN:
            print(pygame.key.name(event.key))
    keys = pygame.key.get_pressed()   
	#char = screen.getch()
    if keys[pygame.K_p]:
        break
    elif keys[pygame.K_UP]:
        print("up")
        leftMotor.forward(motorSpeed)
        rightMotor.forward(motorSpeed)
    elif keys[pygame.K_DOWN]:
        print("down")
        leftMotor.backward(motorSpeed)
        rightMotor.backward(motorSpeed)
    elif keys[pygame.K_RIGHT]:
        print("right")
        leftMotor.forward(motorSpeed)
        rightMotor.backward(motorSpeed)
    elif keys[pygame.K_LEFT]:
        print("left")
        leftMotor.backward(motorSpeed)
        rightMotor.forward(motorSpeed)
    elif keys[pygame.K_s]:
        print("stop") 
        leftMotor.stop() 
        rightMotor.stop()  
    elif keys[pygame.K_i]:
        motorSpeed += 0.1
        if motorSpeed > 1:
            motorSpeed = 1
        print(motorSpeed)
    elif keys[pygame.K_u]:
        motorSpeed -= 0.1
        if motorSpeed < 0.5:
            motorSpeed = 0.5
        print(motorSpeed)
    elif keys[pygame.K_q]:
        leftServo.value = -1
    elif keys[pygame.K_w]:
        leftServo.value = 0
    elif keys[pygame.K_e]:
        leftServo.value = 1
    elif keys[pygame.K_r]:
        rightServo.value = 1
    elif keys[pygame.K_t]:
        rightServo.value = 0
    elif keys[pygame.K_y]:
        rightServo.value = -1
    if abs((gyro[2] - gyroCal[2])/12.5) > 2:
        cummulativeAngle += ((gyro[2] - gyroCal[2])/12.5)*pow((gyroTimeNew - gyroTimeOld),2)
    print(cummulativeAngle)
    print("")

		
print("Closed")
#Close down curses properly, inc turn echo back on!
    


while False:
	
	
	#print("Acc: %.4f, %.4f, %.4f" % (acc))
	
	#print("Gyro: %.4f, %.4f, %.4f" % (gyro))
	#print("Yaw: %.4f" % (gyro[2] - gyroCal[2]))
	print("Elapsed: %.4f" % (gyroTimeNew - gyroTimeOld))
	if abs((gyro[2] - gyroCal[2])/125.0) > 2:
		cummulativeAngle += ((gyro[2] - gyroCal[2])/125.0)*pow((gyroTimeNew - gyroTimeOld),2)
	
	print((gyro[2] - gyroCal[2])/125.0)
	print(cummulativeAngle)
	print("")
	#time.sleep(0.01)
	#continue
