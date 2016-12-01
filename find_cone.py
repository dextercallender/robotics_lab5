import cv2
import numpy as np
import picamera
from gopigo import *
from time import *
try:
    import wx
except ImportError:
    raise ImportError,"The wxPython module is required to run this program"
import atexit
atexit.register(stop)

mean = [178.0, 198.0, 249.0]
std = [0.0, 3.0, 1.0]
img = None
camera = picamera.PiCamera()

def initial_left_turn():
    angle = 90.0 + 11.25
    enc_count = angle / 11.25
    enable_encoders()
    enc_tgt(0,1,int(enc_count))
    while (read_enc_status() == 1):
        left_rot()
    stop()
    disable_encoders()

def small_turn_right():
    enable_encoders()
    enc_tgt(1,0,1)
    while (read_enc_status() == 1):
        right_rot()
    stop()
    disable_encoders()
    
def binarize_image(image, mean, stddev):
    '''
        Binarize the image based off of the given mean
        color and stddev
        @param image - rgb image to binarize
        @param mean - the hsv mean of the color to select
        @param stddev - the hsv stddev of the color
        @return - binarized version of the image
    '''
    # Calculate the bounds of the colors
    lower_bound = np.array([mean[0] - 4*stddev[0], mean[1] - 4*stddev[1], \
                   mean[2] - 4*stddev[2]])
    upper_bound = np.array([mean[0] + 4*stddev[0], mean[1] + 4*stddev[1], \
                   mean[2] + 4*stddev[2]])
    # Convert image to hsv
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Get pixels within the range and then threshold it
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    ret, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    # Guassian blur, erosion, dilation to clean up and then enhance image
    gaussian_blur = cv2.GaussianBlur(mask, (3, 3), 0)
    kernel = np.ones((3,3), np.uint8)
    erosion = cv2.erode(gaussian_blur, kernel, iterations=1)
    dilation = cv2.dilate(erosion, kernel, iterations=1)
    return dilation

def resize(image):
    '''
        Cut the given image size in half for faster
        processing
        @param image - image to cut in half
        @param return - image as half size
    '''
    return cv2.resize(image, (0,0), fx=0.5, fy=0.5)

def get_area_and_left_right(image):
    '''
        Given a binarized image find the largest object's
        area and left/right bounds
        @param image - binarized image with object
        @return area - area of object
        @return left - left bound of object
        @return right - right bound of object
    '''
    try:
        # Get connected components
        connectivity = 8
        output = cv2.connectedComponentsWithStats(image, 8, cv2.CV_32S)
        num_components = output[0]
        stats = output[2]
        max_area = 0
        left = 0
        right = 0
        # Find largest connected component as our object
        for i in range(1, num_components):
            if stats[i, cv2.CC_STAT_AREA] > max_area:
                # Set as the area, left, right
                max_area = stats[i, cv2.CC_STAT_AREA]
                left = stats[i, cv2.CC_STAT_LEFT]
                right = left + stats[i, cv2.CC_STAT_WIDTH]
        return max_area, left, right
    except:
        #No connected components found
        return 0, 0, 0
    
def find_cone():
    global mean, std
    initial_left_turn()
    found = False
    curr_area = 0
    for i in range(0, 180/11.25):
        camera.capture(image_name)
        camera_image = cv2.imread(image_name, cv2.IMREAD_COLOR)
        img = resize(camera_image)
        binarized = binarize_image(img, mean, stddev)
        curr_area, left, right = get_area_and_left_right(binarized)
        if curr_area > 100 and img.shape[1]/2 < right and img.shape[1] / 2 < left:
            found = True
            break
    if found:
        while curr_area < 500:
            enable_encoders()
            enc_tgt(1,1,9)
            while(read_enc_status() == 1):
                fwd()
            stop()
            disable_encoders()
            camera.capture(image_name)
            camera_image = cv2.imread(image_name, cv2.IMREAD_COLOR)
            img = resize(camera_image)
            binarized = binarize_image(img, mean, stddev)
            curr_area, left, right = get_area_and_left_right(binarized)
            if curr_area < 100:
                print "Cone lost. Stopping"
                break
    else:
        print "Cone not found :("
