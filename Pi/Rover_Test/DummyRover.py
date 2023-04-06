import curses
import RPi.GPIO as GPIO
import MotorClass as MC

screen = curses.initscr()
curses.noecho() 
curses.cbreak()
screen.keypad(True)

GPIO.setmode(GPIO.BOARD)
#GPIO.setup(11,GPIO.OUT)
#GPIO.setup(13,GPIO.OUT)
#GPIO.setup(29,GPIO.OUT)
#GPIO.setup(31,GPIO.OUT)
#GPIO.setup(12,GPIO.OUT)
#GPIO.output(37,True)

M1 = MC.Motor(11,13)
M2 = MC.Motor(29,31)

try:
    while True:   
        char = screen.getch()
        if char == ord('q'):
            GPIO.cleanup()
            break
        elif char == curses.KEY_UP:
            print("up")
            M1.forward(20)
            M2.forward(20)
        elif char == curses.KEY_DOWN:
            print("down")
            M1.backward(20)
            M2.backward(20)
        elif char == curses.KEY_RIGHT:
            print("right")
            #GPIO.output(11,False)
            #GPIO.output(13,True)
            #GPIO.output(29,True)
            #GPIO.output(31,False)
        elif char == curses.KEY_LEFT:
            print("left")
            #GPIO.output(11,True)
            #GPIO.output(13,False)
            #GPIO.output(29,False)
            #GPIO.output(31,True)
        elif char == 10:
            print("stop")   
            M1.stop()
            M2.stop()
            
finally:
    #Close down curses properly, inc turn echo back on!
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()

