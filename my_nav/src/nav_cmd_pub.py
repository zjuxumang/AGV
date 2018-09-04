#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''nav_cmd ROS Node'''
# license removed for brevity
from __future__ import print_function
#import roslib; roslib.load_manifest('nav_cmd')
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys, select, termios , tty

msg = """
请通过键盘发送指令
-------------------
A 启动送餐路线1
S 启动送餐路线2
D 启动送餐路线3
B 启动通知巡游播报
Y 键盘遥控模式
-------------------

按CTRL+C退出
"""
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin],[],[],0)
    key=sys.stdin.read(1)
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('current_cmd', String, queue_size=10)
    pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('nav_cmd', anonymous=True)

    try:
        print(msg)
        pub.publish('\x08') #客户端启动
        while(1):
            key=getKey()
            if key == 'A':
                print("cmd=A")
            elif key == 'B':
                print("cmd=B")
            else:
                print("unkown cmd")
                if(key== '\x03'):
                    break

            cmd=String()
            cmd.data=key
            pub.publish(cmd)
    except rospy.ROSInterruptException:
        pass
    finally:
        print("退出控制客户端......")
        pub.publish('\x09')
