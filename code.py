from gpiozero import DistanceSensor
import cv2
from picamera2 import Picamera2
import time 
import numpy as np
import RPi.GPIO as GPIO

IN1 = 13
IN2 = 17
IN3 = 26
IN4 = 22

ENA = 16
ENB = 6

S1 = 3 # Green
S2 = 4 # Blue
S3 = 15 # Brown
S4 = 18 # Purple Yello

GPIO.setmode(GPIO.BCM)

ballcoords = [0, 0]
rectanglesize = [0,0]
middle = 320
buffer = 100

ultrasonic = DistanceSensor(echo=14, trigger=2)
camra = Picamera2()
camra.configure(camra.create_preview_configuration(main = {"format":"BGR888", "size": (640, 480)}))

camra.start()
time.sleep(1)

GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
GPIO.setup([S1, S2, S3, S4], GPIO.OUT)

GPIO.output(ENA, GPIO.HIGH)
GPIO.output(ENB, GPIO.HIGH)

cruising = 80
overdrive = 100

pwm = GPIO.PWM(ENA, 50)
pwm2 = GPIO.PWM(ENB, 50)

topleft = GPIO.PWM(S1, 50)
topright = GPIO.PWM(S2, 50)
bottomleft = GPIO.PWM(S3, 50)
bottomright = GPIO.PWM(S4, 50)

topleft.start(0)
topright.start(0)
bottomleft.start(0)
bottomright.start(0)

pwm.start(overdrive)
pwm2.start(cruising)

memory = "right"

rectarea = 0

suspicious = False

global status
status = "readyup"

log = []

global start
global end 

start = time.time()
end = 1000

log = []

def start_time():
    return time.time()

def elapsed():
    end = time.time()
    elapsed = end - start
    return elapsed

def maskon(dframe):
    global redmask
    lower = np.array([0, 0, 150])   # Bound range for red color
    upper = np.array([80, 80, 255])  

    # lower = np.array([150, 0, 0]) # If you want a BLUE ball 
    # upper = np.array([255, 100, 100])  

    redmask = cv2.inRange(dframe, lower, upper) # For color in range, highlight

    (h,w) = redmask.shape


    return redmask
def largestcontour(dmask):
            
    contours, hierarchy = cv2.findContours(dmask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    contourslist = []
    
    for cnt in contours:
        contourslist.append(cnt)
        
    if(len(contourslist) > 0):
        maxcontour = contourslist[0]
        maxarea = cv2.contourArea(contourslist[0])

        
        for i in range(len(contourslist)):
            nowarea = cv2.contourArea(contourslist[i])
            if (nowarea > maxarea):
                maxarea = nowarea
                maxcontour = contourslist[i]

        return maxcontour
def drawrectangle():
    global frame
    global center_x
    global center_y

    frame = camra.capture_array()
    frame = np.asarray(frame)
    height = frame.shape[0]
    width = frame.shape[1]

    center_x = 0
    center_y = 0
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    maskred = maskon(frame)

    conter = largestcontour(maskred)
    
    (x,y,w,h) = cv2.boundingRect(conter)
    
    if (w * h >= 3000):
        cv2.rectangle(frame, (x,y), (x+w,y+h), (0, 255, 0), 4)
        suspicious = False
    
    else:
        cv2.rectangle(frame, (x,y), (x+w,y+h), (0, 0, 255), 2)
        suspicious = True

    ballcoords[0] = x+w/2
    ballcoords[1] = y+h/2

    rectanglesize[0] = w
    rectanglesize[1] = h
    
    rectarea = w * h

def balldetected():
    if (rectanglesize[0] * rectanglesize[1] >= 3000):
        return True
    return False
def ballfound(distances):
    return ((distances <= 0.18) and balldetected())
def readyup():
    print("Gearing Up...")
    topleft.ChangeDutyCycle(7)
    time.sleep(0.1)
    topright.ChangeDutyCycle(7)
    time.sleep(0.1)
    bottomleft.ChangeDutyCycle(7)
    time.sleep(0.1)
    bottomright.ChangeDutyCycle(7)
    time.sleep(0.1)

def pickup():
    print("picking up...")
    bottomleft.ChangeDutyCycle(12)
    bottomright.ChangeDutyCycle(2)
    
    time.sleep(1)
    
    topleft.ChangeDutyCycle(11.2)
    topright.ChangeDutyCycle(2.8)
    
    time.sleep(1)
    
    bottomleft.ChangeDutyCycle(6)
    bottomright.ChangeDutyCycle(8)

    time.sleep(2)
def freeze():
    print("FREEZE!")
    topleft.ChangeDutyCycle(0)
    bottomleft.ChangeDutyCycle(0)
    topright.ChangeDutyCycle(0)
    bottomright.ChangeDutyCycle(0)


def moveforward():
        
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
def turnleft():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
def turnright():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def powersystemshutdownwithoverride():
    GPIO.cleanup

found = True

readyup()
time.sleep(0.5)
freeze()

while True:
    
    #print(f"Area: {rectarea}")
    #print(f"Sus: {suspicious}")
    print(f"Status: {status}")

    if (balldetected() == True and suspicious == False):    
        if (ballcoords[0] <= middle and ballcoords[0] > 0):
            memory = "left"

        elif (ballcoords[0] >= middle and ballcoords[0] != 0):
            memory = "right"
        

    drawrectangle()
    cv2.imshow('mask', redmask)
    cv2.imshow("draw",frame)

    #print("Drawing Rectangle...")
    #print(f"memory: {memory}")
    distance = ultrasonic.distance
    #print(distance)

    if (balldetected() == True and found == True):
        xvalue = ballcoords[0]
        print(f"ball coordinates: {ballcoords}")
    
        if (xvalue <= middle - buffer):
            print("LEFT")
            pwm.ChangeDutyCycle(100)
            pwm2.ChangeDutyCycle(100)
            turnleft()

        elif (xvalue >= middle + buffer):
            print("RIGHT")
            pwm.ChangeDutyCycle(100)
            pwm2.ChangeDutyCycle(100)
            turnright()
        
        elif (xvalue <= middle + buffer and xvalue >= middle - buffer and distance >= 0.1):
            print("FORWARD")
            pwm.ChangeDutyCycle(100)
            pwm2.ChangeDutyCycle(100)
            moveforward()
        
        elif (xvalue > middle + buffer or xvalue < middle - buffer or distance < 0.1):
            stop()
            
    elif (balldetected() == True and found == False):
        found = True
        print("Enemy Spotted!")
        stop()
        time.sleep(0.5)     

    else:
        found = False
        print("Searching...")
        pwm.ChangeDutyCycle(80)
        pwm2.ChangeDutyCycle(80)
        
        if (memory == "left"):
            turnleft()
        else:
            turnright()
    
    if (ballfound(distance)):
        print("Ball Found!")
        stop()
        time.sleep(0.1)    
        pickup()
        freeze()
        time.sleep(1)
        bottomleft.ChangeDutyCycle(12)
        bottomright.ChangeDutyCycle(2)
        time.sleep(2)
        readyup()
        time.sleep(0.5)
        freeze()
    
            
    if(cv2.waitKey(1) & 0xff == ord('q')):
        stop()
        readyup()
        topleft.ChangeDutyCycle(3)
        topright.ChangeDutyCycle(11)
        time.sleep(2.5)
        camra.release()
        cv2.destroyAllWindows()
        break
