#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''cmd_manager ROS Node'''
import rospy
import os
from std_msgs.msg import String
from std_srvs.srv import *

def callback(cmd):
    '''cmd_manager Callback Function'''
    if cmd.data == 's':
        print('S')
        os.system('play ~/catkin_ws/main_startup.wav')          #启动底盘控制器和雷达
        os.system('roslaunch robot_bringup start.launch &')
    elif cmd.data == 'm':                                       #gmapping建图
        print('M')
        os.system('roslaunch my_nav gmapping.launch &')
    elif cmd.data == 'savemap':                                 #保存地图
        os.system('rosrun my_nav saveMap.sh')
    elif cmd.data == 'exitgmapping':                            #退出gmapping不保存
        os.system('rosnode kill /slam_gmapping')
    elif cmd.data =='n':                                        #启动导航包
        os.system('~/catkin_ws/src/my_nav/scripts/exit.sh') #如果导航包已经在运行，则先退出再重启
        os.system('roslaunch my_nav start_nav.launch &')
        os.system('play ~/catkin_ws/nav_startup.wav')
    elif cmd.data == 'r':
        os.system('rosnode kill -a')
    elif cmd.data == 'd':
        os.system('play ~/catkin_ws/line1.wav')
    elif cmd.data == 'b':
        os.system('play ~/catkin_ws/ad_startup.wav')
    elif cmd.data == '1':
        os.system('play ~/catkin_ws/arrive1.wav')
    elif cmd.data == '2':
        os.system('play ~/catkin_ws/arrive2.wav')
    elif cmd.data == '3':
        os.system('play ~/catkin_ws/arrive3.wav')
    elif cmd.data == '4':
        os.system('play ~/catkin_ws/arrive4.wav')
    elif cmd.data == '5':
        os.system('play ~/catkin_ws/arrive5.wav')
    elif cmd.data == "adv":
        os.system('play ~/catkin_ws/adv1.wav')
def listener():
    '''cmd_manager Subscriber'''
    rospy.init_node('cmd_manager')
    print "Ready to service."
    rospy.Subscriber("current_cmd", String, callback,queue_size=1)
    os.system('play ~/catkin_ws/welcome.wav >/dev/null')
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    is_mb_running=0
    listener()
