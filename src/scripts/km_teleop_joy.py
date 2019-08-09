#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    twist = Twist()
    #print data.axes
    left_right = data.axes[0]
    if left_right<0.1 and left_right>-0.1:
        left_right = 0
    top_down = data.axes[1]
    if top_down<0.1 and top_down>-0.1:
        top_down = 0
    twist.linear.x = top_down
    twist.angular.z = left_right
    pub.publish(twist)

def start():
    rospy.init_node('km_dolly_teleop_joy')
    global pub
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    rospy.Subscriber('/joy',Joy,callback)
    rospy.spin()

if __name__ == '__main__':
    start()
