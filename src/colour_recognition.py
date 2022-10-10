#!/usr/bin/env python

import rospy
from task2.msg import ColorRecognition
import pickle
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def callback_color_recognition(image):

    try:
        cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
        imgHSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([imgHSV], [0, 1, 2], None, [8, 8, 8], [0, 256, 0, 256, 1, 256])
        hist = cv2.normalize(hist, hist).flatten()
        result = loaded_model.predict([hist])
        #rospy.loginfo("red: %f, green: %f, blue: %f, black: %f", result[0][0], result[0][1], result[0][2], result[0][3])

        msg = ColorRecognition()
        msg.red = result[0][0]
        msg.green = result[0][1]
        msg.blue = result[0][2]
        msg.black = result[0][3]
        p.publish(msg)

    except CvBridgeError as e:
        rospy.logerr(e)


if __name__ == "__main__":
    rospy.init_node('color_recognition_server')
    s = rospy.Subscriber('color_recognition', Image, callback_color_recognition)
    p = rospy.Publisher('color_probability', ColorRecognition, queue_size = 10)
    global loaded_model
    loaded_model = pickle.load(open('/home/pikanogavicka/ROS/src/task2/src/finalized_colour_model.sav', 'rb'))
    rospy.loginfo("Starting colour recognition node.")
    rospy.spin()
