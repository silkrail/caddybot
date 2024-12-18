#!/usr/bin/env python
#-*- coding:utf-8 -*-	# 한글 주석을 달기 위해 사용한다.

# 단순 입력을 Terminal 에 반환하는 노드
import rospy
from std_msgs.msg import Char
import sys, select
import tty, termios

def getkey():
    
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) 
    return key

if __name__ == '__main__' : 
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('practice_node')
    pub = rospy.Publisher('/ReturnGetkey' , Char , queue_size=10)
    char = Char()

    try : 
        while not rospy.is_shutdown():
            key = getkey()
            if key == '\x03':
                rospy.loginfo("Good bye!")
                break
            elif key != '' : 
                char.data = key
                rospy.loginfo("your key is : %c", char.data)
                
            
            pub.publish(char)
    except :
        pass
