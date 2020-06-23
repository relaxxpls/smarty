#!/usr/bin/env python
import cv2
import imutils
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()
rospy.init_node('smartytbt_select_object')

def nothing(n):
    pass

cv2.namedWindow("Choose the object")
cv2.createTrackbar("Hue Start", "Choose the object", 0, 179, nothing)
cv2.createTrackbar("Hue End", "Choose the object", 179, 179, nothing)
cv2.createTrackbar("Saturation Start", "Choose the object", 0, 255, nothing)
cv2.createTrackbar("Saturation End", "Choose the object", 255, 255, nothing)
cv2.createTrackbar("Value Start", "Choose the object", 0, 255, nothing)
cv2.createTrackbar("Value End", "Choose the object", 255, 255, nothing)
cv2.createTrackbar("0 = Discard Selection\n1 = Save Selection", "Choose the object", 0, 1, nothing)

def select_obj(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    img_hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    hs = cv2.getTrackbarPos("Hue Start", "Choose the object")
    ss = cv2.getTrackbarPos("Saturation Start", "Choose the object")
    vs = cv2.getTrackbarPos("Value Start", "Choose the object")
    he = cv2.getTrackbarPos("Hue End", "Choose the object")
    se = cv2.getTrackbarPos("Saturation End", "Choose the object")
    ve = cv2.getTrackbarPos("Value End", "Choose the object")
    key = cv2.getTrackbarPos("0 = Discard Selection\n1 = Save Selection", "Choose the object")

    mask = cv2.inRange(img_hsv, np.array([hs, ss, vs], dtype=np.uint8), np.array([he, se, ve], dtype=np.uint8))
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    color_mask = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow("Select your Object (mask)", color_mask)

    contour = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = imutils.grab_contours(contour)
    largest_contour = max(contour, key=cv2.contourArea)
    area_default = cv2.contourArea(largest_contour)

    if key == 1:
        with open("smarty_settings.dat", 'w') as file:
            file.write(hs+" "+ss+" "+vs+"\n"+he+" "+se+" "+ve+"\n"+area_default)
        rospy.loginfo("Changes Saved!")
    else:
        rospy.loginfo("Save Your Changes!")

def main():
    rospy.Subscriber("smartytbt/camera1/image_raw", Image, select_obj)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

