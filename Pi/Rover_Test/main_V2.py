from smbus import SMBus
import datetime, threading, time
import math
from gpiozero import Button, Servo, Motor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
import numpy as np
import LSM6DSO
import pygame
from multiprocessing import Queue

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
#pygame.init()
#window = pygame.display.set_mode((300, 300))

##### Motors
#rightMotorSpeed = 0.62
#leftMotorSpeed = 0.62
rightMotorSpeed = 0.57
leftMotorSpeed = 0.57
rightMotor = Motor(26,19)
leftMotor = Motor(21,20)

# Encoder Current Value
rightEncoderVal = 0
leftEncoderVal = 0

# Previous Encoder Value
prevRightStep = 0
prevLeftStep = 0

# Encoder PID
prevRightStepError = 0
sumRightStepError = 0
prevLeftStepError = 0
sumLeftStepError = 0

rightEncoderBuf = 0
leftEncoderBuf = 0


dist = 0
distSetPoint = 9000

ppr = 341
rightEncoder = RotaryEncoder (10, 9, max_steps = 0, pin_factory = factory)
leftEncoder = RotaryEncoder (22, 27, max_steps = 0, pin_factory = factory)

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


# ENCODER UPDATES



def getEncoder():
    global move, turn, rightEncoderBuf, leftEncoderBuf, rightEncoderVal, leftEncoderVal
    if move or turn:
        rightEncoderVal = rightEncoder.steps - rightEncoderBuf
        leftEncoderVal = leftEncoder.steps - leftEncoderBuf
    else:
        rightEncoderBuf = rightEncoder.steps
        leftEncoderBuf = leftEncoder.steps

def stepCalibrate(Kp, Ki, Kd, currCount, prevCount, presetCount, prevError, sumError):
    # +ve = CCW rotation
    diff = presetCount - (abs(currCount) - abs(prevCount))
    #print("Corrected: %0.04f, %0.04f, %0.04f" %(yawKp*distDiff, yawKd*(distDiff-prevDistDiff), yawKi*sumDistDiff))
    correction = (Kp*diff + Kd*(diff-prevError) + Ki*sumError)
    #print("Correction: %.4f, %.4f, %.4f" % (Kp*diff, Kd*(diff-prevError), Ki*sumError))
    prevError = diff
    sumError += diff
    sumError = max(min(200, sumError), -200)
    print("Step Size: %.4f, %.4f" % (diff, (currCount - prevCount)))
       
    return correction, prevError, sumError
        
def distTravel(dist, rightEncoderVal, leftEncoderVal, ppr):
    dist = (rightEncoderVal + leftEncoderVal)*251.33/(2*ppr)
    return dist
	
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
turn = 1
right = 0
left = 0

prevTime = 0
currTime = 0

underSpeedCount = 0

speedPreset = 33
turnSetPoint = 90
turnAngle = 0

while True:
    currTime = time.time()
    '''
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
        #print("up")
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
        leftMotorSpeed = 0.75
        rightMotorSpeed = 0.75
    '''
    if gyroRead or accRead:
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

      
    if (currTime-prevTime) >= 0.05:
        print("")
        #print("Time: %0.04f"%(currTime-prevTime))
        
        getEncoder()
        
        prevTime = currTime
        
        if distSetPoint - dist > 50:
            move = 1
            if distSetPoint - dist < 500:
                speedPreset = 0
            if distSetPoint - dist < 200:
                speedPreset = 0
        else:
            print("Within last 100mm")
            
            move = 0
        print("Turn Decision: %.4f" % (turnSetPoint + cummulativeAngle ))
        
        if turn:  
            if turnSetPoint + cummulativeAngle > 9:
                right = 1
                left = 0
            elif turnSetPoint + cummulativeAngle < -9:
                left = 1
                right = 0
            else:
                print("Turn else")
                turn = 0
                left = 0
                right = 0
                turnSetPoint = 0
        else: 
            cummulativeAngle = 0
       
        
        if 0:
            # Kp, Ki, Kd, currCount, prevCount, presetCount, prevError, sumError
            
            if (rightEncoderVal - prevRightStep > 0) and (leftEncoderVal - prevLeftStep > 0):
                if (rightEncoderVal - prevRightStep > speedPreset*0.5) and (leftEncoderVal - prevLeftStep > speedPreset*0.5):
                    rightCorrection = stepCalibrate(0.00004, 0.00000, 0.00003, rightEncoderVal, prevRightStep, max(min(99, (speedPreset - (rightEncoderVal - leftEncoderVal)*0.25)), 0), prevRightStepError, sumRightStepError)
                    prevRightStepError = rightCorrection[1]
                    sumRightStepError = rightCorrection[2]
                    rightMotorSpeed += rightCorrection[0]
                
                    leftCorrection = stepCalibrate(0.00004, 0.00000, 0.00003, leftEncoderVal, prevLeftStep, max(min(99, (speedPreset + (rightEncoderVal - leftEncoderVal)*0.25)), 0), prevLeftStepError, sumLeftStepError)
                    prevLeftStepError = leftCorrection[1]
                    sumLeftStepError = leftCorrection[2]
                    leftMotorSpeed += leftCorrection[0]
                    
                    underSpeedCount = 0
                
                underSpeedCount += 1
                
                if underSpeedCount >= 5:
                    print ("Under Speed")
                    rightCorrection = stepCalibrate(0.00005, 0, 0.00001, rightEncoderVal, prevRightStep, max(min(99, (speedPreset - (rightEncoderVal - leftEncoderVal)*0.05)), 0), prevRightStepError, sumRightStepError)
                    prevRightStepError = rightCorrection[1]
                    sumRightStepError = rightCorrection[2]
                    rightMotorSpeed += rightCorrection[0]
                
                    leftCorrection = stepCalibrate(0.00005, 0, 0.00001, leftEncoderVal, prevLeftStep, max(min(99, (speedPreset + (rightEncoderVal - leftEncoderVal)*0.05)), 0), prevLeftStepError, sumLeftStepError)
                    prevLeftStepError = leftCorrection[1]
                    sumLeftStepError = leftCorrection[2]
                    leftMotorSpeed += leftCorrection[0]
                    
                    #underSpeedCount = 0
                
            else:			
                print("Unmoved")
                rightCorrection = stepCalibrate(0.00005, 0, 0, rightEncoderVal, prevRightStep, max(min(99, (speedPreset - (rightEncoderVal - leftEncoderVal)*0.05)), 0), prevRightStepError, sumRightStepError)
                prevRightStepError = rightCorrection[1]
                sumRightStepError = rightCorrection[2]
                rightMotorSpeed += rightCorrection[0]
                
                leftCorrection = stepCalibrate(0.00005, 0, 0, leftEncoderVal, prevLeftStep, max(min(99, (speedPreset + (rightEncoderVal - leftEncoderVal)*0.05)), 0), prevLeftStepError, sumLeftStepError)
                prevLeftStepError = leftCorrection[1]
                sumLeftStepError = leftCorrection[2]
                leftMotorSpeed += leftCorrection[0]
                       
            print("Set Speed: %.4f, %.4f" % (max(min(99, (speedPreset - (rightEncoderVal - leftEncoderVal)*0.05)), 0), (max(min(99, (speedPreset + (rightEncoderVal - leftEncoderVal)*0.05)), 0))))
            
            
            rightMotorSpeed = max(min(0.75, rightMotorSpeed), 0.48)
            #print("%.6f" % rightMotorSpeed)
            leftMotorSpeed = max(min(0.75, leftMotorSpeed), 0.48)
            
            leftMotor.forward(leftMotorSpeed)
            rightMotor.forward(rightMotorSpeed)
            print("Speed, left = % .08f, right = % .08f" % (leftMotorSpeed, rightMotorSpeed))
            print("%0.04f" % (rightEncoderVal - leftEncoderVal))
                        
        elif right:
            print("Turning Right")
            leftMotor.forward(leftMotorSpeed)
            rightMotor.backward(rightMotorSpeed)
        elif left:
            print("Turning Left")
            leftMotor.backward(leftMotorSpeed)
            rightMotor.forward(rightMotorSpeed)

        else:
            leftMotor.stop() 
            rightMotor.stop() 
            
        prevRightStep = rightEncoderVal
        prevLeftStep = leftEncoderVal
        turnAngle = ((abs(rightEncoderVal)*282.74/(ppr))/(1319.47))*360
        #print("Turn Angle: %.4f degree" % turnAngle)
        #print("Left Turn Angle: %.4f degree" % (((abs(leftEncoderVal)*282.74/(ppr))/(1319.47))*360))
        #print("Right: %.4f" % rightEncoderVal)
        #print("Left: %.4f" % leftEncoderVal)
        #print("Travelled: %.4f" % ((abs(rightEncoderVal) + leftEncoderVal)*282.74/(2*ppr)))
        print("Gyro Angle: %.4f deg" % cummulativeAngle)
        #dist = distTravel(dist, rightEncoderVal, leftEncoderVal, ppr)
        #print("Distance Travelled: %.2f mm" % dist)
            #print("Encoder Count %0.04f, %0.04f" % (prevRightStep, prevLeftStep))
            #print("%0.04f" % (rightEncoder.steps))
            
    #print("%0.04f" % (rightEncoder.steps))
    #print("%0.04f" % (rightEncoderVal))
    
    
   
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
