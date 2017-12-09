#!/usr/bin/env python
#(Shebang to ensure the interpreter used is the first in the environment's $PATH)


# GIZMO PIXEL - Interactive robotic pet code
# Version 50 6/12/17
# Ben Cobley and Joe Shepherd November 2017

# MODULES

from __future__ import division  # change the / operator to mean true division

print('GIZMO PIXEL - COPYRIGHT BEN COBLEY AND JOE SHEPHERD NOVEMBER 2017')

print('Importing SimpleCV...')  # import optical tracking
import SimpleCV
print('Importing OMXPLAYER...')  # import video player
from omxplayer.player import *
print('Importing pathlib...')  # import object oriented file system paths
from pathlib import Path
print('Importing GPIO...')  # import GPIO control
import RPi.GPIO as GPIO
print('Importing time...')
import time  # import time
print('Importing math...')
import math  # import mathS
print('Importing numpy...')
import numpy  # import numpy
print('Importing random...')
import random  # import random
print('Importing LED driver...')
import Adafruit_WS2801   # import LED drivers
import Adafruit_GPIO.SPI as SPI # import Serial Peripheral Interface for GPIO

# SETTINGS

showDisplay = False  # turn OpenCV display on/off
showAnimation = True  # turn Gizmo face animation display on on/off

# INITIALISATION

print('Initialising camera...')
cam = SimpleCV.Camera()

print('Initialising motors...')
GPIO.setmode(GPIO.BCM)  # use broadcom pin numbers
GPIO.setwarnings(False)
GPIO.setup(25, GPIO.OUT)  # right forward
GPIO.setup(17, GPIO.OUT)  # right backward
GPIO.setup(12, GPIO.OUT)  # right PWM
GPIO.setup(22, GPIO.OUT)  # left forward
GPIO.setup(23, GPIO.OUT)  # left backward
GPIO.setup(13, GPIO.OUT)  # left PWM
l = GPIO.PWM(12, 50)  # set pwm pin and frequency
r = GPIO.PWM(13, 50)  # set pwm pin and frequency
l.start(0)   # initialise pin
r.start(0)   # initialise pin

normalSpeed = 20  # speed of normal driving motion

print('Initialising servo...')
GPIO.setup(18, GPIO.OUT)  # Front leg
s = GPIO.PWM(18, 50)  # set pwm pin and frequency
s.start(11)   # initialise pin

upDuty = 11  # set duty cycle of up position
downDuty = 9.5  # set duty cycle of down position

print('Initialising LEDS...')
PIXEL_COUNT = 16  # number of leds
PIXEL_CLOCK = 21  # LED clock pin number
PIXEL_DOUT = 10  # LED DOUT pin number
pixels = Adafruit_WS2801.WS2801Pixels(PIXEL_COUNT, clk=PIXEL_CLOCK, do=PIXEL_DOUT)


print('Resurrecting...')
dead = False  # bringing gizmo back to life
print('Feeding...')
hunger = 0  # eating children
print('Resting...')
tiredness = 0  # counting sheep
print('Entertaining...')
boredom = 0  # watching netflix

# VARIABLES

print('Setting variables...')
count = 0
lostCount = 0
prevCount, prevColour, prevCoords,  = None, None, None
botCoords = None
lost = False

green = (0, 255, 0)  # RGB colour values for detection
gThresh = 170  # binarizing threshold (see below). Values found by calibration testing
blue = (0, 0, 255)
bThresh = 150
orange = (255, 0, 0)
oThresh = 160
red = (255, 0, 0)
rThresh = 150

sBotMin = 1600  # dot areas (in number of pixels) for detection - based on size and distance from camera
sBotMax = 2800
lBotMin = 4000
lBotMax = 7000
xsFloorMin = 450
xsFloorMax = 800
ballMin = 3500
ballMax = 5500

## MAIN

# OPTICAL TRACKING

# SimpleCV is a simplified, accessible version of OpenCV, where much of OpenCV's customisable functionality is condensed
# into logical simple Python functions. However this unfortunately does not mean to say that is is easy to work with!
# This code was adapted from examples on the SimpleCV github and from Practical Computer Vision with SimpleCV:
# K Demaagd et al. ISBN: 9781449320362.

# These examples showed how to identify and track individual circles, but could not differentiate between multiple circles.
# Multiple object tracking, colour, and size differentiation was entirely my own work - Ben Cobley.

def update_coords(colour, threshold, minArea, maxArea, count):
    global prevCount
    global prevColour
    global circles

    if colour != prevColour or count != prevCount:  # to decrease optical tracking update times, don't repeat for scan for tracking of same colour (unless new image)
        cdimg = img.colorDistance(colour).dilate(2)  # stretch colours so that chosen colour is darker
        binimg = cdimg.binarize(threshold, 255)  # binarise to separate colours darker than threshold from lighter than threshold
        blobs, circles, dots = None, None, None  # reset circles found to None
        blobs = binimg.findBlobs()  # find 'blobs' in image - areas of consistent colour
        if blobs:
            circles = blobs.filter([blob.isCircle(0.25) for blob in blobs])  # filter to find circles in blobs - below theshold - experimented to find 0.25 as optimum
    prevColour = colour  # update current colour
    prevCount = count  # update current count
    if circles:
        dots = circles.filter([minArea < circle.area() < maxArea for circle in circles])  # filter for circles of correct area - define as dots
        if dots:
            coords = (dots[-1].x, dots[-1].y)  # output dot coordinates
            return coords
        else:
            return None
    else:
        return None


def move_forward(target, speed):
    global botCoords
    motionVector = tuple(numpy.subtract(target, botCoords))  # use bot coords to find vector to target
    moveTime = math.sqrt(motionVector[0] ** 2 + motionVector[1] ** 2) / 150  # time calculated by distance to target - found by experimentation
    drive(speed)  # call drive function
    time.sleep(moveTime) # wait
    motors_off()  # turn motors off
    return moveTime  # output time so that gizmo can reverse same distance


def move_backward(moveTime):
    drive(-normalSpeed)   # call drive function
    time.sleep(moveTime)  # wait
    motors_off()  # turn motors off


def move_angle(target):
    global botCoords
    if angle_test(target) is False:  # test to see whether angle is already correct
        motionVector = tuple(numpy.subtract(target, botCoords))  # calculate vector to target
        motionAngle = - round(math.degrees(math.atan2(motionVector[1], motionVector[0])))  # calculate angle to target
        turnAngle = -(botAngle + motionAngle)  # angle required to turn
        if turnAngle < -180:
            turnAngle = turnAngle + 360  # change angle coordinate system to be in range -180 to 180
        if turnAngle > 25:  # turn speed based on turn angle
            right(-normalSpeed)
            left(normalSpeed)
            time.sleep(0.3)  # keep motors on for 0.3 seconds or until next optical tracking update
            motors_off()
        elif 25 >= turnAngle > 15:
            right(-normalSpeed)
            left(normalSpeed)
            time.sleep(0.05)
            motors_off()

        elif turnAngle  < -25:
            right(normalSpeed)
            left(-normalSpeed)
            time.sleep(0.3)
            motors_off()
        elif -25 <= turnAngle < -15:
            right(normalSpeed)
            left(-normalSpeed)
            time.sleep(0.05)
            motors_off()


def drive(value):
    right(value)
    left(value)


def right(value):  # define functions 'right' and 'left' to make calling motors easier. 'Value' gives motor PWM and direction
    global rightMotor
    rightMotor = abs(value)  # uses absolute value to define PWM 
    r.ChangeDutyCycle(rightMotor)
    if value >= 0:  # uses positivity/negativity to define direction
        GPIO.output(17, True)  
        GPIO.output(25, False)
    elif value < 0:
        GPIO.output(17, False)
        GPIO.output(25, True)


def left(value):
    global leftMotor
    leftMotor = abs(value)  # uses absolute value to define PWM 
    l.ChangeDutyCycle(leftMotor)
    if value >= 0:  # uses positivity/negativity to define direction
        GPIO.output(22, True)
        GPIO.output(23, False)
    elif value < 0:
        GPIO.output(22, False)
        GPIO.output(23, True)


def motors_off():
    GPIO.output(25, False)
    GPIO.output(17, False)
    GPIO.output(22, False)
    GPIO.output(23, False)


def look_up():  # move servo to 'up' position 
    print('up')
    for repeats in range(1, 3):  # repeated 3 times because does not reach desired angle in 1 attempt - too much load
        s.ChangeDutyCycle(upDuty)
        time.sleep(0.1)


def look_down():  # move servo to 'down' position 
    print('down')
    for repeats in range(1, 3):  # repeated 3 times because does not reach desired angle in 1 attempt - too much load
        s.ChangeDutyCycle(downDuty)
        time.sleep(0.1)


def servo_sleep(length):  # up and down 'breathing' motion while gizmo is asleep 
    print('sleep')
    look_up()
    duty = upDuty  # set initial position
    count = 0
    while count < length:  # run for length of sleep face animation
        print('in')
        while duty > downDuty:
            duty -= ((upDuty - downDuty) / 10)  # move from up to down in 10 1 tenth motions
            s.ChangeDutyCycle(duty)
            time.sleep(0.1)
        print('out')
        while duty < upDuty:
            duty += ((upDuty - downDuty) / 10)  # move from down to up in 10 1 tenth motions
            s.ChangeDutyCycle(duty)
            time.sleep(0.1)
        count += 1


def blink_color(pixels, blink_times, color):  # adapted from the Adafruit WS2801 Python GitHub
    for i in range(blink_times):  # number of blinks
        for j in range(pixels.count()):  # number of LEDs
            pixels.set_pixel(j, Adafruit_WS2801.RGB_to_color( color[0], color[1], color[2] ))
        pixels.show()
        time.sleep(0.5)  # blink length


def stationary_test(coords):  # used to test whether object is stationary from one frame to the next  //
    global prevCoords         # to improve reliability of tracking and stop tracking before object has finished //
    if prevCoords:            # being placed into the box
        if coords[0] - prevCoords[0] < 4 and coords[1] - prevCoords[1] < 4:  # compare current frame coordinates to coords in previous frame
            return True
    prevCoords = coords
    return False


def angle_test(target):  # test whether gizmo is facing the correct direction before an action is performed
    global botCoords
    motionVector = tuple(numpy.subtract(target, botCoords))  # calculate vector to target
    motionAngle = - round(math.degrees(math.atan2(motionVector[1], motionVector[0])))  # calculate angle to target
    turnAngle = -(botAngle + motionAngle)  # angle required to turn
    if turnAngle < - 180:
        turnAngle = turnAngle + 360  # change angle coordinate system to be in range -180 to 180
    if abs(turnAngle) < 15:  # within 15 degrees angle test is true
        return True
    else:
        return False


def change_video(path):  # change current face animation video to new one
    if showAnimation is True:  # only if show animation is on
        global player
        try:  # quit current video first - changeover is faster
            player.quit()
        except:  # error handling required because video player runs very poorly, often breaks
            print('No video playing to quit - Change.')
        video_path = Path(path)  # use video path from function call
        player = OMXPlayer(video_path)  # define player
        player.set_video_pos(1, 1, 719, 479)  # define video coords, 1 pixel in from edges of screen
        player.play()  # play video


def try_video(path):  # change video ONLY IF previous video has finished
    global player
    if showAnimation is True:  # only if show animation is on
        try:  # quit current video first - changeover is faster
            player.is_playing()
        except:  # error handling required because function returns true or just breaks - this was an absolute ***** to troubleshoot
            try:
                player.quit()
            except:  # error handling required because video player runs very poorly, often breaks
                print('No video playing to quit - Try.')
            print('Resetting video')
            video_path = Path(path)  # use video path from function call
            player = OMXPlayer(video_path)  # define player
            player.set_video_pos(1, 1, 719, 479)  # define video coords, 1 pixel in from edges of screen
            player.play()  # play video


def stop_video():  # stop video
    global player
    if showAnimation is True:
        try:
            player.is_playing()
        except:  # error handling required because video player runs very poorly, often breaks
            try:
                player.quit()
            except:
                print('No video playing to quit - Try.') #try


blink_color(pixels, blink_times=1, color=(0, 0, 0))  # turn leds off - on at start as standard

while True:  # while true loop to run infinitely
    try:  # except keyboard interrupt, to help quit without failiures
        print('______')
        print(count)
        print('Hunger: ', hunger)  # keep track of gizmo's needs in terminal
        print('Tiredness: ', tiredness)
        print('Boredom: ', boredom)
        img = cam.getImage().flipHorizontal()  # get image from webcam
        sBlue, lBlue, lGreen, xsRed, xsBlue, ball = None, None, None, None, None, None  # reset tracked dots to none

        # UPDATE COORDS
        # update coords of all dots based on their colour, colour threshold and size thresholds
        sBlue = update_coords(blue, bThresh, sBotMin, sBotMax, count)
        lBlue = update_coords(blue, bThresh, lBotMin, lBotMax, count)
        xsBlue = update_coords(blue, bThresh, xsFloorMin, xsFloorMax, count)
        lGreen = update_coords(green, gThresh, xsFloorMin, xsFloorMax, count)
        xsRed = update_coords(red, rThresh, xsFloorMin, xsFloorMax, count)
        ball = update_coords(orange, oThresh, ballMin, ballMax, count)

        # MOTORS

        if lBlue and sBlue:  # perform all below operations ONLY if lBlue and sBlue are found - if Gizmo is tracking correctly
            botCoords = lBlue  # bot coords defined by large blue dot on forehead
            botVector = (lBlue[0] - sBlue[0], lBlue[1] - sBlue[1])  # calculate vector of gizmo
            botAngle = round(math.degrees(math.atan2(botVector[1], botVector[0])))  # calculate angle to horizontal

            # in elif loop so that only one of the following functions is performed for each camera refresh

            if dead and lGreen and xsBlue and xsRed:  # resurrect if all 3 dots are placed in the camera view
                tiredness = 0
                boredom = 0
                hunger = 0
                dead = False
                change_video('/home/pi/Happy.mp4')
                look_up()
                print('Resurrecting...')
            elif tiredness > 1000 and hunger > 1000 and boredom > 1000:  # die if gizmo's needs get too high
                look_down()
                print('DEAD - you did not look after me properly!')
                if dead is False:
                    change_video('/home/pi/Dead.mp4')
                dead = True

            elif xsBlue and tiredness < 500 and boredom > 5:  # dance if boombox is placed in camera view
                print('Boombox found')
                stationary = stationary_test(xsBlue)  # only once boombox is stationary
                if stationary:
                    print('Boombox stationary')
                    change_video('/home/pi/Disco.mp4')
                    right(-25)
                    left(25)
                    time.sleep(0.25)
                    for repeats in range(0, 6):  # move left and right and flash led lights
                        right(25)
                        left(-25)
                        blink_color(pixels, blink_times=1, color=(255, 0, 0))  # use time.sleep in blink function for timing
                        right(-25)
                        left(+25)
                        blink_color(pixels, blink_times=1, color=(0, 255, 0))
                        right(25)
                        left(-25)
                        blink_color(pixels, blink_times=1, color=(0, 0, 255))
                        right(-25)
                        left(+25)
                        blink_color(pixels, blink_times=1, color=(255, 255, 0))
                        right(25)
                        left(-25)
                        blink_color(pixels, blink_times=1, color=(255, 0, 255))
                        right(-25)
                        left(+25)
                        blink_color(pixels, blink_times=1, color=(0, 255, 255))
                        motors_off()
                    right(25)
                    left(-25)
                    time.sleep(0.25)
                    drive(-normalSpeed)  # reverse back to back of box
                    time.sleep(1.5)
                    motors_off()
                    blink_color(pixels, blink_times=1, color=(0, 0, 0))
                    change_video('/home/pi/Happy.mp4')
                    tiredness += 200
                    hunger += 200
                    boredom = 0

            elif ball and boredom > 5:  # 'kick' ball if ping pong ball is placed in camera view and bored enough
                if stationary_test(ball):  # only once ball is stationary
                    if angle_test(ball):  # if gizmo is facing ball
                        change_video('/home/pi/Activated.mp4')
                        time.sleep(0.5)
                        print('FIRE')
                        drive(100)  # drive at full speed
                        time.sleep(0.5)
                        motors_off()
                        time.sleep(1)
                        tiredness += 200
                        hunger += 200
                        boredom = 0
                        move_backward(1)
                    else:
                        move_angle(ball)

            elif lGreen and tiredness > 5:  # sleep if bed is placed in camera view and tired enough
                if stationary_test(lGreen):
                    if angle_test(lGreen):
                        print('Going to bed')
                        change_video('/home/pi/Sleep.mp4')
                        moveTime = move_forward(lGreen, normalSpeed)
                        servo_sleep(4)
                        tiredness = 0
                        hunger += 200
                        boredom += 200
                        move_backward(moveTime)
                    else:
                        move_angle(lGreen)

            elif xsRed and hunger > 2:  # eat if food bowl is placed in camera view and hungry enough
                if stationary_test(xsRed):
                    if angle_test(xsRed):
                        print('Going to eat')
                        change_video('/home/pi/Eating.mp4')
                        moveTime = move_forward(xsRed, normalSpeed)
                        time.sleep(27)  # change to length of vid
                        hunger = 0
                        try_video('/home/pi/Happy.mp4')
                        look_up()
                        move_backward(moveTime)
                    else:
                        move_angle(xsRed)

            elif tiredness > 500:  # if tiredness gets too high: display tired face and run idle motion
                print('Tired')
                look_down()
                try_video('/home/pi/Tired.mp4')
                if count % 15 is True:
                    right(20)
                    left(-20)
                    time.sleep(0.5)
                    motors_off()
                    time.sleep(0.5)
                    right(-20)
                    left(20)
                    time.sleep(0.5)
                    motors_off()
                elif count % 16 is True:
                    right(-20)
                    left(20)
                    time.sleep(0.5)
                    motors_off()
                    time.sleep(0.5)
                    right(20)
                    left(-20)
                    time.sleep(0.5)
                    motors_off()

            elif hunger > 700:  # if hunger gets too high: display angry face and run idle motion
                print('Hungry')
                look_up()
                try_video('/home/pi/Angry.mp4')
                if count % 15 is True:  # run only once every 15 refreshes
                    right(20)
                    left(-20)
                    time.sleep(0.5)
                    motors_off()
                    time.sleep(0.5)
                    right(-20)
                    left(20)
                    time.sleep(0.5)
                    motors_off()

            elif boredom > 250:  # if boredom gets too high: display sad face and run idle motion
                print('Bored')
                look_up()
                try_video('/home/pi/Sad.mp4')
                if count % 15 is True:  # run only once every 15 refreshes
                    right(20)
                    left(-20)
                    time.sleep(0.5)
                    motors_off()
                    time.sleep(0.5)
                    right(-20)
                    left(20)
                    time.sleep(0.5)
                    motors_off()

            # IDLE

            else:  # if no other actions occur - call idle motion
                print('Idle')
                try_video('/home/pi/Idle.mp4')
                look_up()
                if count % 15 is True:  # run only once every 15 refreshes
                    right(20)
                    left(-20)
                    time.sleep(0.5)
                    motors_off()
                    time.sleep(0.5)
                    right(-20)
                    left(20)
                    time.sleep(0.5)
                    motors_off()


        elif botCoords is None:  # print until bot is found for first time
            print('Finding bot for first time...')
        else:  # run lost protocol
            try_video('/home/pi/Lost.mp4')
            motors_off()
            print('Gizmo is lost, searching... ')

        # DISPLAY

        if showDisplay is True:  # if show display is on, show SimpleCV object tracking on screen to aid troubleshooting
            if xsBlue:
                img.drawCircle(xsBlue, 10, blue, 5)
            if sBlue:
                img.drawCircle(sBlue, 10, blue, 5)
            if lBlue:
                img.drawCircle(lBlue, 10, blue, 5)
            if xsRed:
                img.drawCircle(xsRed, 10, red, 5)
            if lGreen:
                img.drawCircle(lGreen, 10, green, 5)
            if ball:
                img.drawCircle(ball, 10, orange, 5)
            img.show()

        hunger = hunger + random.randint(0, 4)  # randomly increase Gizmo's needs as time goes on
        boredom = boredom + random.randint(0, 4)
        tiredness = tiredness + random.randint(0, 4)
        count = count + 1

    except KeyboardInterrupt:  # keyboard interrupt handling to prevent errors/crashing upon quit
        print(' Quitting... ')
        GPIO.output(25, False)  # turn motors off or they will continue running forever
        GPIO.output(17, False)
        GPIO.output(22, False)
        GPIO.output(23, False)
        l.stop()
        r.stop()
        GPIO.cleanup()
        try:  # wait till video finishes to prevent fatal OMXPlayer crashes
            if player.is_playing():
                print('Waiting for video to stop...')
        except:
            try:
                player.quit()
            except:
                print('This video player runs like pure ****')  # yes, this was very frustrating
        print('Thank you for interacting with Gizmo')  # thank you for reading all the way to the bottom of my code
        quit()


'''
Once Gizmo was tested in his tiny box and we discovered the split ring was not possible, we realised we would have to
dramatically simplify the code to have him sit at the back and drive back-and-forth to interact. This prevented
him from strangling himself on the wire and from spending too much time facing backwards. We are still planning to 
install a USB slip ring for the final version, which will allow him to have much more life and character. Code was 
written that may be brought back for this version:

Removed code:
Fluid motion - driving to a given target in one motion - no need to change angle first/constantly stop to wait for 
tracking update.
Destination check - check whether Gizmo is at destination before performing action
Random driving while idling - randomly driving to targets around the box to keep himself entertained while not 
being interacted with (caused strangulation)
'''
