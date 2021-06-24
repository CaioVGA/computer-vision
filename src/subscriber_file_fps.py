#! /usr/bin/env python

import rospy
from std_msgs.msg import String

def callback_function(frame_update):
    rospy.loginfo(frame_update.data)

if __name__ == "__main__":
    
    rospy.init_node('subscriber_frames')
    rospy.Subscriber("/dados_frames", String, callback_function)
    rospy.spin()