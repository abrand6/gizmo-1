# GIZMO PIXEL - Interactive robotic pet code
# Version 50 6/12/17
# Ben Cobley and Joe Shepherd November 2017

#Utils.py : functions file 



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
