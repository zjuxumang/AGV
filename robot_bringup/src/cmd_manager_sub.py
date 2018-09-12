#!/usr/bin/env python
'''cmd_manager ROS Node'''
import rospy
import os
from std_msgs.msg import String
from std_srvs.srv import *

def callback(cmd):
    '''cmd_manager Callback Function'''
    if cmd.data == 's':
        print('S')
        os.system('roslaunch robot_bringup start.launch &')
        os.system('play ~/catkin_ws/startup.wav')
    elif cmd.data == 'm':
        print('M')
        os.system('roslaunch my_nav gmapping.launch &')
    elif cmd.data == 'savemap':
        os.system('rosrun my_nav saveMap.sh')
    elif cmd.data == 'exitgmapping':
        os.system('rosnode kill /slam_gmapping')
    elif cmd.data =='n':
        os.system('roslaunch my_nav start_nav.launch &')
        os.system('play ~/catkin_ws/nav_startup.wav')

def listener():
    '''cmd_manager Subscriber'''
    rospy.init_node('cmd_manager')
    print "Ready to service."
    rospy.Subscriber("current_cmd", String, callback,queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
