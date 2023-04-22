from smbus import SMBus
import datetime, threading, time
import math
from gpiozero import Button, Servo, Motor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
import numpy as np
import LSM6DSO
import pygame

####################### SETUP ####################### 

factory = PiGPIOFactory()

##### IMU
i2cbus = SMBus(1)
LSM6DSOAddr = 0x6B
INT1 = Button(13, pull_up = False, bounce_time = 0.001, pin_factory = factory)  
INT2 = Button(6, pull_up = False, bounce_time = 0.001, pin_factory = factory)  
acc = (0.0, 0.0, 0.0)
gyro = (0.0, 0.0, 0.0)
gyroRead = 0
accRead = 0

cummulativeAngle = 0.0
yawAcc = 0.0

prevDistDiff = 0
sumDistDiff = 0

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
rightMotorSpeed = 0.7
leftMotorSpeed = 0.7
rightMotor = Motor(26,19)
leftMotor = Motor(21,20)

ppr = 341
rightEncoder = RotaryEncoder (10, 9, max_steps = 0)
leftEncoder = RotaryEncoder (27, 22, max_steps = 0)

##### Charging Plates

leftServo = Servo (12, min_pulse_width = 0.0005, max_pulse_width = 0.0025, pin_factory = factory)
rightServo = Servo (16, min_pulse_width = 0.0005, max_pulse_width = 0.0025, pin_factory = factory)

####################### FUNCTIONS ####################### 

##### IMU
def LSM6DSO_readAcc():
	global acc, accRead
	accRead = 1
	x = LSM6DSO.readAccX(LSM6DSOAddr, i2cbus)
	y = LSM6DSO.readAccY(LSM6DSOAddr, i2cbus)
	z = LSM6DSO.readAccZ(LSM6DSOAddr, i2cbus)
	acc = (x, y, z)
	return (x, y, z)
	
def LSM6DSO_readGyro():
	global gyro, gyroTimeNew, gyroTimeOld, gyroRead
	gyroRead = 1;
	gyroTimer = time.time()
	x = LSM6DSO.readGyroX(LSM6DSOAddr, i2cbus)
	y = LSM6DSO.readGyroY(LSM6DSOAddr, i2cbus)
	z = LSM6DSO.readGyroZ(LSM6DSOAddr, i2cbus)
	gyroTimeOld = gyroTimeNew
	gyroTimeNew = time.time()
	#print("Gyro Read")
	gyro = (x, y, z)
	return (x, y, z)
	
def IMU_Gyro_Cal():
	global gyroRead, gyro
	gyroCal = []
	i = 0
	while i < 100:
		if gyroRead:
			gyroCal.append(gyro)
			gyroRead = 0
			i += 1
			print(i)
	x = 0.0;
	y = 0.0;
	z = 0.0;
	for j in range(len(gyroCal)):
		temp = gyroCal[j]
		x += temp[0]
		y += temp[1]
		z += temp[2]
	
	return (x/(i+1), y/(i+1), z/(i+1))

def IMU_Acc_Cal():
	global accRead, acc
	AccCal = []
	i = 0
	while i < 100:
		if accRead:
			AccCal.append(acc)
			accRead = 0
			i += 1
			print(i)
	x = 0.0;
	y = 0.0;
	z = 0.0;
	for j in range(len(AccCal)):
		temp = AccCal[j]
		x += temp[0]
		y += temp[1]
		z += temp[2]
	
	return (x/(i+1), y/(i+1), z/(i+1))
	
INT1.when_pressed = LSM6DSO_readAcc
INT2.when_pressed = LSM6DSO_readGyro

yawKp = 0.00004
yawKd = 0.00002
yawKi = 0.000015

# ENCODER UPDATES
def speedCalibrate():
    global leftMotorSpeed, rightMotorSpeed, prevDistDiff, sumDistDiff
    # +ve = CCW rotation
    distDiff = rightEncoder.steps + leftEncoder.steps
    rightMotorSpeed -= (yawKp*distDiff + yawKd*prevDistDiff + yawKi*sumDistDiff)
    leftMotorSpeed += (yawKp*distDiff + yawKd*prevDistDiff + yawKi*sumDistDiff)
    prevDistDiff = distDiff
    sumDistDiff += distDiff
    print(rightEncoder.steps)
    print(leftEncoder.steps)
    #print(distDiff)
    rightMotorSpeed = max(min(0.85, rightMotorSpeed), 0.65)
    leftMotorSpeed = max(min(0.85, leftMotorSpeed), 0.65)
        
	
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
accCal = IMU_Acc_Cal();
print("Gyro calibrated: %.04f, %.04f, %.04f" % gyroCal)
print("Acc calibrated: %.04f, %.04f, %.04f" % accCal)

move = 0

#timerThread = threading.Thread(target=speedCalibrate)
#timerThread.daemon = True
#timerThread.start()

prevTime = 0
currTime = 0


while True:
    currTime = time.time()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
        if event.type == pygame.KEYDOWN:
            print(pygame.key.name(event.key))
    keys = pygame.key.get_pressed()   
	#char = screen.getch()
    if keys[pygame.K_p]:
        print("stop")
        break
    elif keys[pygame.K_UP]:
        print("up")
        move = 1
        leftMotor.forward(leftMotorSpeed)
        rightMotor.forward(rightMotorSpeed)
    elif keys[pygame.K_DOWN]:
        print("down")
        leftMotor.backward(leftMotorSpeed)
        rightMotor.backward(rightMotorSpeed)
    elif keys[pygame.K_RIGHT]:
        print("right")
        cummulativeAngle = 0
        leftMotor.forward(leftMotorSpeed)
        rightMotor.backward(rightMotorSpeed)
    elif keys[pygame.K_LEFT]:
        print("left")
        cummulativeAngle = 0
        leftMotor.backward(leftMotorSpeed)
        rightMotor.forward(rightMotorSpeed)
    elif keys[pygame.K_s]:
        leftMotor.stop() 
        rightMotor.stop() 
        move = 0 
    elif keys[pygame.K_q]:
        leftServo.value = -0.5
        rightServo.value = -0.5
    elif keys[pygame.K_w]:
        leftServo.value = 0
        rightServo.value = 0
    elif keys[pygame.K_e]:
        leftServo.value = 0.1
        rightServo.value = 0.1
    elif keys[pygame.K_r]:
        cummulativeAngle = 0
        leftMotorSpeed = 0.6
        rightMotorSpeed = 0.6
    if gyroRead or accRead and ~keys[pygame.K_LEFT] and ~keys[pygame.K_RIGHT]:
        gyroYaw = ((gyro[2] - gyroCal[2]))*(gyroTimeNew - gyroTimeOld)/10
        temp = round(math.floor(-(acc[1]-accCal[1])*100)/100.0,1)
        temp2 = round(math.floor((acc[0]-accCal[0])*100)/100.0,1)
        if temp == 0 or temp2 == 0:
            accYaw = 0
        else:
            accYaw = 180 * math.atan2(temp, temp2) / math.pi
        cummulativeAngle = (cummulativeAngle + gyroYaw)*1 + accYaw*0
        gyroRead = 0
        accRead = 0
        
        # ~ if cummulativeAngle < -1:
           # ~ rightMotorSpeed += 0.001
           # ~ leftMotorSpeed -= 0.001
        # ~ elif cummulativeAngle > 1:
            # ~ leftMotorSpeed += 0.001
            # ~ rightMotorSpeed -= 0.001
        # ~ if rightMotorSpeed > 1:
            # ~ rightMotorSpeed = 1
        # ~ if rightMotorSpeed < 0.6:
            # ~ rightMotorSpeed = 0.6
        # ~ if leftMotorSpeed > 1:
            # ~ leftMotorSpeed = 1
        # ~ if leftMotorSpeed < 0.6:
            # ~ leftMotorSpeed = 0.6
        # ~ print(leftMotorSpeed)
        # ~ print(rightMotorSpeed)

            
    if move == 1:
        if (currTime-prevTime) >= 0.01:
            print("%0.04f"%(currTime-prevTime))
            prevTime = currTime
            speedCalibrate()
            leftMotor.forward(leftMotorSpeed)
            rightMotor.forward(rightMotorSpeed)
            print("left = % .02f" % leftMotorSpeed)
            print("right = % .02f" % rightMotorSpeed)
        
    
    print("%0.04f" % (rightEncoder.steps + leftEncoder.steps))
   
    #print("")

print("Closed")
#Close down curses properly, inc turn echo back on!
    


while False:
	
	
	#print("Acc: %.4f, %.4f, %.4f" % (acc))
	
	#print("Gyro: %.4f, %.4f, %.4f" % (gyro))
	#print("Yaw: %.4f" % (gyro[2] - gyroCal[2]))
	#print("Elapsed: %.4f" % (gyroTimeNew - gyroTimeOld))
	if (abs((gyro[2] - gyroCal[2])) > 1 and gyroRead):
		print((gyro[2] - gyroCal[2])*(gyroTimeNew - gyroTimeOld))
		cummulativeAngle += (gyro[2] - gyroCal[2])*(gyroTimeNew - gyroTimeOld)/10
		gyroRead = 0
	
	#print((gyro[2] - gyroCal[2]))
	print(cummulativeAngle)
	print("")
	#time.sleep(0.01)
	#continue
