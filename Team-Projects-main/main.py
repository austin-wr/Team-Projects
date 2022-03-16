import cv2
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

import MachineVision

# Set to true for debugging
debug_mode = False

# Program variables
pickup_activated = False
robot_stop = False
robot_state = "WAIT"
color = "GREEN"
camera = PiCamera()
camera.resolution = (2592, 1944)
camera.framerate = 15
# rawCapture = PiRGBArray(camera)


def log_state_information(info):
    print("LOG: ", end='')
    print(info)


def pickup_animation():
    print("TODO: Pickup animation")


while not robot_stop:
    # Read commands from input file, update robot_state & color
    # Commands:
    # - "GO *COLOR*"
    # - "STOP"

    # Add special character to end of commands to prevent reading an unfinished command.

    # Call log_state_information() with current state & color

    # Call check_state(robot_state)
    # - if "WAIT" then continue loop (sleep as well)
    # - if "GO" then get image and move
    # - if "STOP" then update robot_stop and break

    # Get image from camera
    camera.start_preview()
    time.sleep(5)
    camera.capture("/home/pi/Desktop/python_test.jpg")
    camera.stop_preview()
    image = cv2.imread("/home/pi/Desktop/python_test.jpg")
    cv2.imshow("Initial", image)

    # If a QR code has already been found we can't pick up something else
    if not pickup_activated:
        # Pass the image to QR reader, update pickup_activated if QR code is present
        pickup_activated = MachineVision.detect_qr_code(debug_mode, image)

        # Call pickup_animation() if QR code was found
        if pickup_activated:
            pickup_animation()

        # Call log_state_information() with pickup_activated
        log_state_information("pickup_activated: " + str(pickup_activated))

        # TESTING - Stop the program if a QR code has been detected
        robot_stop = True

    # Pass the image to line reader
    angle = MachineVision.detect_line(debug_mode, image, color)

    # Call move(angle) with angle from line reading
    log_state_information("angle: " + str(angle))


# Remove debug windows
# cv2.destroyAllWindows()
