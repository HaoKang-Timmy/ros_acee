#!/usr/bin/python3
import rospy
from std_msgs.msg import Float64
from pynput.keyboard import Key, Listener
import geometry_msgs.msg

publeft = None
pubright = None  
def callback(data):
 
    publeft.publish((data.linear.x+data.angular.z*0.1)/0.08)     
    pubright.publish((data.linear.x-data.angular.z*0.1)/0.08)
def listener():
    global publeft
    global pubright
    rospy.init_node('listener', anonymous=True)
    publeft=rospy.Publisher('/course_agv/left_wheel_velocity_controller/command', Float64, queue_size=1)
    pubright=rospy.Publisher('/course_agv/right_wheel_velocity_controller/command', Float64, queue_size=1)
    rospy.Subscriber('/course_agv/velocity', geometry_msgs.msg.Twist, callback)
    rospy.spin()







if __name__ == '__main__':
    listener()