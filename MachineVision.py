import cv2
import numpy as np


def draw_bounding_box(img, points):
    # Reformat [[(x1,y1),(x2,y2),(x3,y3),(x4,y4)]] -> [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
    points = points[0]

    # Draw the bounding box
    for i in range(len(points)):
        pt1 = [int(val) for val in points[i]]
        pt2 = [int(val) for val in points[(i + 1) % 4]]
        cv2.line(img, pt1, pt2, color=(0, 255, 255), thickness=3)

    cv2.imshow('Detected QR code', img)


def detect_qr_code(debug, img):
    qr_decoder = cv2.QRCodeDetector()

    # Detect & decode QR code
    # _ for unused results
    data, points, _ = qr_decoder.detectAndDecode(img)

    if len(data) > 0:
        if debug:
            draw_bounding_box(img, points)
            print("Decoded Data : {}".format(data))
        return True
    else:
        return False


def identify_color_channels_given_desired_color(img, color):
    b_channel, g_channel, r_channel = cv2.split(img)

    if color == "BLUE":
        return b_channel, r_channel, g_channel
    elif color == "RED":
        return r_channel, g_channel, b_channel
    elif color == "GREEN":
        return g_channel, b_channel, r_channel
    else:
        print("MachineVision> ERROR: Given invalid color (must be RED, BLUE, or GREEN)")
        return None


def apply_blur(selected_color, side_color1, side_color2):
    selected_color = cv2.GaussianBlur(selected_color, (13, 13), cv2.BORDER_DEFAULT)
    side_color1 = cv2.GaussianBlur(side_color1, (13, 13), cv2.BORDER_DEFAULT)
    side_color2 = cv2.GaussianBlur(side_color2, (13, 13), cv2.BORDER_DEFAULT)
    return selected_color, side_color1, side_color2


def contrast_equalization(selected_color, side_color1, side_color2, clip_limit, tile_grid_size):
    clahe = cv2.createCLAHE(clipLimit, tileGridSize)
    selected_color = clahe.apply(selected_color)
    side_color1 = clahe.apply(side_color1)
    side_color2 = clahe.apply(side_color2)
    return selected_color, side_color1, side_color2


def subtract_size_colors_and_equalize(selected_color, side_color1, side_color2):
    test_image = cv2.subtract(selected_color, side_color1)
    test_image = cv2.subtract(test_image, side_color2)
    return cv2.equalizeHist(test_image)


def find_max_index_of_contours(contours):
    running_index = 0
    running_area = 0
    max_index = 0

    while running_index < len(contours):
        c_area = cv2.contourArea(contours[running_index])
        if c_area > running_area:
            running_area = c_area
            max_index = running_index
        running_index += 1

    return max_index


def detect_line(debug, img, color):
    if debug:
        cv2.imshow("Straight Line Test - Original", img)

    selected_color, side_color1, side_color2 = identify_color_channels_given_desired_color(img, color)

    # Blur it to get rid of high frequency
    selected_color, side_color1, side_color2 = apply_blur(selected_color, side_color1, side_color2)

    # Do a contrast limited adaptive histogram equalization.
    # The parameters are tweaked to expand out and blend noise.
    selected_color, side_color1, side_color2 = contrast_equalization(selected_color, side_color1, side_color2, clip_limit=20.0, tile_grid_size=(2,2))

    if debug:
        cv2.imshow("selected channel", selectedColor)
        cv2.imshow("side1", sideColor1)
        cv2.imshow("side2", sideColor2)

    # Here we're going a step further than separating colors by color channel.
    # Since some colors may be a blend of R, G, and/or B, you can isolate the pure color
    # by subtracting each channel from the selected channel. We then equalize this to bring out the
    # gradations left
    selected_color = subtract_side_colors_and_equalize(selected_color, side_color1, side_color2, debug)

    # This is forcing anything above 20 (0-255 range) to be full white and anything below that to be full black
    _, selected_color = cv2.threshold(selected_color, 20, 255, cv2.THRESH_BINARY)

    if debug:
        cv2.imshow("adap", selectedColor)

    # For debugging merge our grayscale image into RGB in order to draw our contour and display it for testing
    merged_image = cv2.merge((selected_color, selected_color, selected_color))

    # Find contours from the threshold image. This is a vector outline of the threshold image.
    contours, hierarchy = cv2.findContours(selected_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(merged_image, contours, -1, (25, 212, 140), 2)

    # Search contour list for largest contour. Multiple contours can be found if there is noise in the image
    # or stray colored items on the floor. Here we're just wanting the largest, which is likely our tape.
    max_index = find_max_index_of_countours(contours)

    # Draw contour onto our image for testing
    if debug:
        cv2.drawContours(merged_image, [contours[max_index]], -1, (25, 212, 140), 2)
        cv2.imshow("Contour", mergedImage)

    # Get rotated min area bounding box.
    rect = cv2.minAreaRect(contours[max_index])
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    # Draw rotated bounding box
    if debug:
        cv2.drawContours(merged_image, [box], 0, (255, 0, 255), 2)
        cv2.imshow("Bound Box", mergedImage)

    # Angle of rotation. This is limited as past 90 degrees what is a "rotated" rectangle becomes ambiguous
    return rect[2]