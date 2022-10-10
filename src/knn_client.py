#!/usr/bin/env python

import sys
import rospy
from task2.srv import knn

def knn_client():
    rospy.wait_for_service('knn_service')
    link = "http://box.vicos.si/rins/b.txt"
    x = 9; y = 2
    try:
        service = rospy.ServiceProxy('knn_service', knn)
        resp1 = service(link, x, y)
        print(resp1.color)
        return resp1.color
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == "__main__":
    rospy.init_node('knn_client', anonymous=True)
    knn_client();

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
