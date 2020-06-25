#!/usr/bin/env python
import cv2
import numpy as np
import imutils
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from smartytbt.msgs import smarty_msgs

control = Twist()
bridge = CvBridge()
rospy.init_node('smarty_object_detection')
pub = rospy.Publisher("/smartytbt/cmd_vel", Twist, queue_size=1)

with open("/home/laxman/catkin_ws/src/smartytbt/scripts/smarty_settings.dat", 'r') as file:
    line = file.readlines()
    str_parts = line[0].split()
    lower_range = np.array([int(str_parts[0]), int(str_parts[1]), int(str_parts[2])], dtype=np.uint8)
    str_parts = line[1].split()
    upper_range = np.array([int(str_parts[0]), int(str_parts[1]), int(str_parts[2])], dtype=np.uint8)
    area_default = float(line[2])

def nothing(n):
    pass

def detect_obj(msg):
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    w = int(frame.shape[1])

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    img_hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(img_hsv, lower_range, upper_range)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contour = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = imutils.grab_contours(contour)
    frame_marked = frame.copy()

    area_current = 0
    centre = 0, 0

    if len(contour) > 0:
        largest_contour = max(contour, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        m = cv2.moments(largest_contour)
        centre = (int(m['m10']/m['m00']), int(m['m01']/m['m00']))
        if radius > 10:
            cv2.circle(frame_marked, (int(x), int(y)), int(radius), (255, 255, 255), 2)
            cv2.circle(frame_marked, centre, 5, (150, 150, 150), -1)
        area_current = cv2.contourArea(largest_contour)
    cv2.imshow("Display", frame_marked)
    
    ratio = area_current/area_default
    control.linear.y = 0
    control.linear.z = 0
    control.angular.x = 0
    control.angular.y = 0

    w = int(w/3)
    x = int(centre[0])
    if x < w:   # turn right
        control.angular.z = -0.5

    elif x > w*2:   # turn left
        control.angular.z = 0.5

    else:
        control.angular.z = 0
        
    if ratio < 0.25:    # move forward
        control.linear.x = 0.5

    elif ratio > 4:     # move behind
        control.linear.x = -0.5

    else:
        control.linear.x = 0
    
    pub.publish(control)
    
    cv2.waitKey(10)

# error = float((cX - 320) / 800)
def main():
    rospy.Subscriber("/smarty_camera/image_raw", Image, detect_obj)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
    
