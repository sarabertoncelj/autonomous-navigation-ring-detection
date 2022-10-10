#!/usr/bin/env python

import sys
import rospy
from task2.srv import qr
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

dims = (0, 0, 0)

def qr_client():
    rospy.wait_for_service('qr_service_py')
    try:
        service = rospy.ServiceProxy('qr_service_py', qr)
        resp1 = service()
        print(resp1.url)
        return resp1.url
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == "__main__":
    rospy.init_node('qr_client', anonymous=True)
    qr_client();

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
