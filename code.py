from gpiozero import DistanceSensor
import cv2
from picamera2 import Picamera2
import time 
import numpy as np
import RPi.GPIO as GPIO

IN1 = 5
IN2 = 4
IN3 = 26
IN4 = 22

ENA = 21
ENB = 20

ballcoords = [0, 0]
rectanglesize = [0,0]
middle = 320
buffer = 75

ultrasonic = DistanceSensor(echo=14, trigger=2)
camra = Picamera2()
camra.configure(camra.create_preview_configuration(main = {"format":"BGR888", "size": (640, 480)}))

camra.start()
time.sleep(1)

GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)

GPIO.output(ENA, GPIO.HIGH)
GPIO.output(ENB, GPIO.HIGH)

pwm = GPIO.PWM(ENA, 5000)
pwm2 = GPIO.PWM(ENB, 5000)

crawling = 100
cruising = 100
overdrive = 100

pwm.start(cruising)
pwm2.start(cruising)

pwm.ChangeDutyCycle(cruising)
pwm2.ChangeDutyCycle(cruising)

def maskon(dframe):
    global redmask
    lower = np.array([0, 0, 150])   # Bound range for red color
    upper = np.array([100, 100, 255])  

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
    cv2.rectangle(frame, (x,y), (x+w,y+h), (0, 255, 0), 2)

    ballcoords[0] = x+w/2
    ballcoords[1] = y+h/2

    rectanglesize[0] = w
    rectanglesize[1] = h

def balldetected():
    if (rectanglesize[0] * rectanglesize[1] >= 10000):
        return True
    
    return False

def ballfound(distances):
    return (distances <= 0.1 and balldetected())



def goslow():
    pwm.ChangeDutyCycle(overdrive)
    pwm2.ChangeDutyCycle(crawling)

def gomedium():
    pwm.ChangeDutyCycle(overdrive)
    pwm2.ChangeDutyCycle(cruising)

def overdrive_activate():
    pwm.ChangeDutyCycle(overdrive)
    pwm2.ChangeDutyCycle(overdrive)



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

while True:
    drawrectangle()
    cv2.imshow('mask', redmask)
    cv2.imshow("draw",frame)

    print("Drawing...")
    distance = ultrasonic.distance
    print(distance)

    if (balldetected() == True):
        print(f"balls: {ballcoords}")
        print(f"middle: {middle}")
        print(f"X dot value: {ballcoords[0]}")
        xvalue = ballcoords[0]
        print(f"xvalue: {xvalue}")

        if (xvalue <= middle - buffer):
            print("LEFT")
            turnleft()

        elif (xvalue >= middle + buffer):
            print("RIGHT")
            turnright()
        
        elif (xvalue <= middle + buffer and xvalue >= middle - buffer and distance >= 0.1):
            print("FORWARD")
            moveforward()
        
        elif (xvalue > middle + buffer or xvalue < middle - buffer or distance < 0.1):
            stop()
            

    else:
        if (balldetected() == False):
            print("Searching... seek and destroy!")
            turnright()
            if (balldetected() == True):
                stop()

    if (ballfound(distance)):
        print("Ball Found!")
        stop()
    
    if(cv2.waitKey(1) & 0xff == ord('q')):
        stop()
        camra.release()
        cv2.destroyAllWindows()
        break
