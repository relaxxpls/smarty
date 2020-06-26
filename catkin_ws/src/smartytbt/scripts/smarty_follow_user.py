#!/usr/bin/env python
import cv2
import numpy as np
import imutils
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from smarty_msgs.msg import data

smarty_data = data()
bridge = CvBridge()

rospy.init_node('smarty_object_detection')
pub = rospy.Publisher("/smarty_tbt/cmd_vel", Twist, queue_size=1)

MAX_LIN_VEL = 3
MAX_ANG_VEL = 3
LIN_VEL_STEP_SIZE = 1
ANG_VEL_STEP_SIZE = 1

def nothing(n):
    pass

def makeSimpleProfile(output, inpu, slop):
    if inpu > output:
        output = min(inpu, output + slop )
    elif inpu < output:
        output = max(inpu, output - slop )
    else:
        output = inpu
    return output

def constrain(inpu, low, high):
    if inpu < low:
        inpu = low
    elif inpu > high:
        inpu = high
    else:
        inpu = inpu
    return inpu
    
def movement(ratio, w, centre_x):
    try:
        control = Twist()
        target_linear_vel = 0.0
        target_angular_vel = 0.0
        control_linear_vel = 0.0
        control_angular_vel = 0.0

        w = int(w/3)
        if centre_x < w:   # turn left
            target_angular_vel = constrain((target_angular_vel - ANG_VEL_STEP_SIZE), -MAX_ANG_VEL, MAX_ANG_VEL)
        elif centre_x > w*2:   # turn right
            target_angular_vel = constrain((target_angular_vel + ANG_VEL_STEP_SIZE), -MAX_ANG_VEL, MAX_ANG_VEL)
        else:
            target_angular_vel = 0

        if ratio < 0.25:    # move forward
            target_linear_vel = constrain((target_linear_vel + LIN_VEL_STEP_SIZE), -MAX_LIN_VEL, MAX_LIN_VEL)
        elif ratio > 4:     # move behind
            target_linear_vel = constrain((target_linear_vel - LIN_VEL_STEP_SIZE), -MAX_LIN_VEL, MAX_LIN_VEL)
        else:
            target_linear_vel = 0

        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
        control.linear.x = control_linear_vel
        control.linear.y = 0.0
        control.linear.z = 0.0

        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        control.angular.x = 0.0
        control.angular.y = 0.0
        control.angular.z = control_angular_vel

        pub.publish(control)

    except Exception as e:
        print(e)

    finally:
        control = Twist()
        control.linear.x = 0.0
        control.linear.y = 0.0
        control.linear.z = 0.0
        control.angular.x = 0.0
        control.angular.y = 0.0
        control.angular.z = 0.0
        pub.publish(control)
    
    
def detect_obj(image_msg, smarty_data_msg):
    frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    w = int(frame.shape[1])
    
    lower_range = np.array([int(smarty_data_msg.hs), int(smarty_data_msg.ss), int(smarty_data_msg.vs)], dtype=np.uint8)
    upper_range = np.array([int(smarty_data_msg.he), int(smarty_data_msg.se), int(smarty_data_msg.ve)], dtype=np.uint8)
    
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
    
    ratio = float(area_current/smarty_data_msg.area_default)
    movement(ratio, w, centre[0])
    cv2.waitKey(10)

def main():
    # rospy.Subscriber("/smarty_camera/image_raw", Image, detect_obj)
    # rospy.Subscriber("/smarty_data", data, detect_obj)
    
    image_sub = message_filters.Subscriber("/smarty_tbt/smarty_camera/image_raw", Image)
    smarty_data_sub = message_filters.Subscriber("/smarty_data", data)
    ts = message_filters.TimeSynchronizer([image_sub, smarty_data_sub], 10)
    ts.registerCallback(detect_obj)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
