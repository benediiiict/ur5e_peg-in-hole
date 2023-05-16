#!/usr/bin/python3

import sys
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

Xfactor = 0.002541
Yfactor = 0.0023456

def coordinate_tf(camera):
    pointx = (480 - camera[1]) * Xfactor
    pointy = (320 - camera[0]) * Yfactor
    coordinate = np.array([pointx, pointy])
    print(coordinate)
    # return 0

def detect_circle(src):
    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    gray = cv.medianBlur(gray, 5)
    
    rows = gray.shape[0]
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8,
                               param1=100, param2=30,
                               minRadius=1, maxRadius=20)
    
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])     # (x, y) coordinate in the screen 
            print(center)             # (306, 232)
            # circle center red
            cv.circle(src, center, 1, (0, 0, 255), 3)
            # circle outline
            radius = i[2]
            cv.circle(src, center, radius, (0, 255, 0), 3)

    coordinate_tf(center)        
    
    # cv.imshow("detected circles", src)
    # cv.waitKey(0)

    return 0


def read_image():
    status = rospy.wait_for_message("/camera/depth/image_raw", Image)      
    status = CvBridge().imgmsg_to_cv2(status, "bgr8")
    # print(status)
    print("===== OK!!! take hole picture =====")
    detect_circle(status)


if __name__ == "__main__":
    rospy.init_node("circle_detect")
    # main(sys.argv[1:])
    read_image()



# def main(argv):
    
#     default_file = '/home/ben/work/ur5e_RL/src/UR_move_test/image_detect/image.png'
#     filename = argv[0] if len(argv) > 0 else default_file

#     # Loads an image
#     src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_COLOR)
#     # Check if image is loaded fine
#     if src is None:
#         print ('Error opening image!')
#         print ('Usage: hough_circle.py [image_name -- default ' + default_file + '] \n')
#         return -1
    
    
#     gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    
    
#     gray = cv.medianBlur(gray, 5)
    
    
#     rows = gray.shape[0]
#     circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8,
#                                param1=100, param2=30,
#                                minRadius=1, maxRadius=20)
    
    
#     if circles is not None:
#         circles = np.uint16(np.around(circles))
#         for i in circles[0, :]:
#             center = (i[0], i[1])
#             # circle center
#             cv.circle(src, center, 1, (0, 0, 255), 3)
#             # circle outline
#             radius = i[2]
#             cv.circle(src, center, radius, (0, 255, 0), 3)
    
#     cv.imshow("detected circles", src)
#     cv.waitKey(0)

#     return 0