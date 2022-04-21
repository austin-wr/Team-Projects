import cv2
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import RPi.GPIO as GPIO
import pygame

import MachineVision

# GPIO Addresses
GPIO.setmode(GPIO.BOARD) # Uses physical pin numbering system

bMKit = MotorKit(address=0x60)  # Bottom MotorKit 
tMKit = MotorKit(address=0x62)  # Top MotorKit
Servo = tMKit.motor3
Lifter = bMKit.stepper2
RWheel = tMkit.stepper1
LWheel = bMkit.stepper1
Infrared = 14
GPIO.setup(Infrared, GPIO.in)


# Calculate arc distance from wheel to wheel distance and rotation angle
def rotateDistanceFromAngle(wheelToWheelDistInches, rotAngle):
    return ((2 * math.pi * wheelToWheelDistInches) * (rotAngle / 360))


# Calculate distance per step based on wheel diameter
# Technically this is static and can be made a constant.
# Only calculated here to give an abstracted self-documented method
def stepToDistanceInches():
    degreePerStep = 1.8
    wheelDiameterInches = 2.55906

    return ((math.pi * wheelDiameterInches) * (degreePerStep / 360))


# Figure out how many steps to rotate a specific amount
def stepsToRotate(rotAngle):
    # Change this to reflect wheel to wheel measurement (from center of wheel)
    wheelToWheelDistInches = 10

    return rotateDistanceFromAngle(wheelToWheelDistInches, rotAngle) / stepToDistanceInches()


def log_state_information(fileName, module, info):
    print(module + " " + str(info))

    # Open the file, write, and close
    file = open(fileName, 'a')
    file.write(module + " " + str(info))
    file.close()


def read_command_file(fileName):
    # Open the file
    file = open(fileName, 'r')

    # Update `line` until we reach end of file
    line = file.readline()
    while line:
        line = file.readline()

    # Command will be the last line present in the file
    file.close()
    return line


def update_color(command):
    parsed_command = command.split("_")
    last_entry = parsed_command[-1]

    match last_entry:
        case "Red":
            return "RED"
        case "Green":
            return "GREEN"
        case "Blue":
            return "BLUE"


def moveRobot(rotAngle, reverseDir):
    # Threshold for determining if robot is at a complete stop
    # and rotate, or if it will move forward/backward while rotating
    rotateThreshold = 60

    # Threshold for not attempting to rotate
    # This will decrease the wiggling of movement
    rotateThresholdLowerBound = 8
    lowerBoundSteps = 5

    moveAndRotate = True
    moveEqually = False

    if abs(rotAngle) >= rotateThreshold:
        moveAndRotate = False

    if abs(rotAngle) <= rotateThresholdLowerBound:
        moveEqually = True

    # We can calculate the steps for a single motor
    # assuming the other wheel is kept stationary...
    # However, forward movement would be terrible with this method.
    # So, we can move forward/backward while rotating. However, if we do this
    # without sampling line tracking in between movement, we may go outside the camera's visible line
    # tracking range. So, if we move while rotating, maybe divide out steps so that we make progress,
    # however, we can still not move the full amount so we can cycle back through the line tracking 
    stepDivisor = 6

    if moveAndRotate == False:
        stepDivisor = 4

    # Primary direction of movement based on rotation angle
    moveLeft = True

    if rotAngle > 0:
        moveLeft = False

    leftWheelDirection = stepper.BACKWARD
    rightWheelDirection = stepper.FORWARD

    if reverseDir == True:
        leftWheelDirection = stepper.FORWARD
        rightWheelDirection = stepper.BACKWARD

    # Code here assumes global registration of two variables, kit1, kit2, outside of function scope
    # These follow the naming convention of the sample code for the motor hats, but can be changed to anything
    # Assume kit1 is left and kit2 is right

    # Move equally means move both wheels the same amount forward/backward
    if moveEqually == True:
        stepTrack = 0
        while stepTrack <= lowerBoundSteps:
            print("Move equally")
            LWheel.onestep(direction=leftWheelDirection)
            RWheel.onestep(direction=rightWheelDirection)
            stepTrack += 1
    else:
        # Else, we're rotating in some fashion
        # stepCount = stepsToRotate(abs(rotAngle))

        # print("Raw step count: " + str(stepCount))

        # stepCount = math.ceil(stepCount/stepDivisor)
        # print("Step count divided: " + str(stepCount))

        # Force either result to an int for motor hat functions
        # stepCount = int(stepCount)

        stepCount = 10

        # Allow both motors to rotate
        # Rotate left by double
        if moveAndRotate == True:
            stepTrack = 0
            while stepTrack <= stepCount:
                if moveLeft == True:
                    print("Rotate left and moving left motor, step: " + str(stepTrack))
                    LWheel.onestep(direction=leftWheelDirection)
                    RWheel.onestep(direction=rightWheelDirection)
                    RWheel.onestep(direction=rightWheelDirection)
                else:
                    print("Rotate right and moving right motor, step: " + str(stepTrack))
                    LWheel.onestep(direction=leftWheelDirection)
                    LWheel.onestep(direction=leftWheelDirection)
                    RWheel.onestep(direction=rightWheelDirection)
                stepTrack += 1
        else:
            # Else, allow only one motor to rotate, moving the full rotation amount
            stepTrack = 0
            while stepTrack <= stepCount:
                if moveLeft == True:
                    print("Rotate left with left motor fixed, step: " + str(stepTrack))
                    RWheel.onestep(direction=rightWheelDirection)
                else:
                    print("Rotate right with right motor fixed, step: " + str(stepTrack))
                    LWheel.onestep(direction=rightWheelDirection)

                stepTrack += 1

    RWheel.release()
    LWheel.release()


def beep(wavFile):
    pygame.mixer.music.load(wavFile)
    pygame.mixer.music.play()

    while pygame.mixer.music.get_busy() == 1:
        continue


def raise_lifter():
    pullTime = 1000
    for i in range(pullTime):
        Lifter.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
    Lifter.release()


def lower_lifter():
    releaseTime = 500
    for i in range(releaseTime)
        Lifter.onestep(direction=stepper.BACKWARD)
    Lifter.release()

    Servo.throttle = 1.0
    time.sleep(0.5)
    Servo.throttle = 0


def pickup_animation():
    lower_lifter()

    distance = 10
    for i in range(distance)
        moveRobot(90, True)

    raise_lifter()


def detect_obstacle():
    return GPIO.input(Infrared) == 1


def main():
    debug_mode = False

    # Program variables
    pickup_activated = False
    robot_stop = False
    color = ""

    # Files
    command_file_name = "commands.txt"
    log_file_name = "stats.txt"
    audio_file = "beep.wav"

    # Audio setup
    pygame_mixer.init()
    speaker_volume = 1
    pygame.mixer.music.set_volume(speaker_volume)

    # Camera
    camera = PiCamera()
    rawCapture = PiRGBArray(camera)

    while not robot_stop:
        if detect_obstacle():
            log_state_information(log_file_name, "Infrared>", "Obstacle detected.")
            beep(audio_file)
            time.sleep(3)
            continue


        # Read command from default command file
        command = read_command_file(command_file_name)

        # Update the color to follow
        color = update_color(command)

        # Log the current color & command
        log_state_information(log_file_name, "Command> ", command)
        log_state_information(log_file_name, "Color> ", color)

        # Detect IR sensor, if found, continue to loop until the item is no longer in the path, beep to alert users, log information to state file

        # Handle the current command
        # Commands:
        #  - "Move To Red"
        #  - "Move To Green"
        #  - "Move To Blue"
        #  - "Stop"
        #  - "Beep"
        #  - "Raise Lifter"
        #  - "Lower Lifter"
        match command:
            case "Stop":
                robot_stop = True
                break
            case "Beep":
                beep()
                continue
            case "Raise_Lifter":
                raise_lifter()
                continue
            case "Lower_Lifter":
                lower_lifter()
                continue
            case "Move_to_Red":
                log_state_information(log_file_name, "Following> ", color)
            case "Move_to_Blue":
                log_state_information(log_file_name, "Following> ", color)
            case "Move_to_Green":
                log_state_information(log_file_name, "Following> ", color)
            case _:
                log_state_information(log_file_name, "Command Parser Error> ", command)
                continue

        # Get image from camera
        camera.start_preview()
        time.sleep(0.5)
        camera.capture(rawCapture, format="bgr")
        image = rawCapture.array
        camera.stop_preview()

        # If a QR code has already been found we can't pick up something else
        if not pickup_activated:
            # Pass the image to QR reader, update pickup_activated if QR code is present
            pickup_activated = MachineVision.detect_qr_code(debug_mode, image)
            log_state_information(log_file_name, "Pickup Status> ", pickup_activated)

            # QR Code was found
            if pickup_activated:
                pickup_animation()

        # Pass the image to line reader
        angle = MachineVision.detect_line(debug_mode, image, color)

        # Log the current angle
        log_state_information(log_file_name, "Angle> ", angle)

        # Move the robot at the specified angle
        robotMove(angle, True)

if __name__ == "__main__":
    main()
