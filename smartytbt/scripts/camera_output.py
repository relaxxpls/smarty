#!/usr/bin/env python3.8
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
rospy.init_node('smarty_camera_output')

def detect_obj(msg):
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow("Detected Object!", frame)

def main():
    rospy.Subscriber("/smartytbt/camera1/image_raw", Image, detect_obj)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
