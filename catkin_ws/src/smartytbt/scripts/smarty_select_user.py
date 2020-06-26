#!/usr/bin/env python
import cv2
import imutils
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from smarty_msgs.msg import data

bridge = CvBridge()
smarty_data = data()
rospy.init_node('smarty_select_object')
area_default = None

def nothing(n):
    pass

def select_obj(msg):
    cv2.namedWindow("Choose the object")
    cv2.createTrackbar("Hue Start", "Choose the object", 0, 179, nothing)
    cv2.createTrackbar("Hue End", "Choose the object", 179, 179, nothing)
    cv2.createTrackbar("Saturation Start", "Choose the object", 0, 255, nothing)
    cv2.createTrackbar("Saturation End", "Choose the object", 255, 255, nothing)
    cv2.createTrackbar("Value Start", "Choose the object", 0, 255, nothing)
    cv2.createTrackbar("Value End", "Choose the object", 255, 255, nothing)
    cv2.createTrackbar("0 = Discard Selection\n1 = Save Selection", "Choose the object", 0, 1, nothing)

    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    img_hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    hs = cv2.getTrackbarPos("Hue Start", "Choose the object")
    ss = cv2.getTrackbarPos("Saturation Start", "Choose the object")
    vs = cv2.getTrackbarPos("Value Start", "Choose the object")
    he = cv2.getTrackbarPos("Hue End", "Choose the object")
    se = cv2.getTrackbarPos("Saturation End", "Choose the object")
    ve = cv2.getTrackbarPos("Value End", "Choose the object")
    key = cv2.getTrackbarPos("0 = Discard Selection\n1 = Save Selection", "Choose the object")

    mask = cv2.inRange(img_hsv, np.array([int(hs), int(ss), int(vs)], dtype=np.uint8), 
                                np.array([int(he), int(se), int(ve)], dtype=np.uint8))
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    color_mask = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow("Select your Object (mask)", color_mask)

    contour = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = imutils.grab_contours(contour)

    if int(key) == 0:
        if len(contour) > 0:
            largest_contour = max(contour, key=cv2.contourArea)
            smarty_data.area_default = int(cv2.contourArea(largest_contour))
    
    smarty_data.hs = hs
    smarty_data.ss = ss
    smarty_data.vs = vs
    smarty_data.he = he
    smarty_data.se = se
    smarty_data.ve = ve
    
    pub = rospy.Publisher('smarty_data', data, queue_size=10)
    pub.publish(smarty_data)
    cv2.waitKey(10)

def main():
    rospy.Subscriber("/smarty_tbt/smarty_camera/image_raw", Image, select_obj)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()

