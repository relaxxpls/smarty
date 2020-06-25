#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
rospy.init_node('smarty_camera_output')

def detect_obj(msg):
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow("Detected Object!", frame)
    cv2.waitKey(10)

def main():
    rospy.Subscriber("/smarty_camera/image_raw", Image, detect_obj)
    try:
    	 rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
