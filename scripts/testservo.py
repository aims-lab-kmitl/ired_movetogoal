#!/usr/bin/python3

import rospy
from std_msgs.msg import Int16
import time

servo_1_ = Int16()
servo_2_ = Int16()
servo_3_ = Int16()

def main():
    rospy.init_node('test_servo_node', anonymous=True)
    servo_1_pub_ = rospy.Publisher('/servo_1_position', Int16, queue_size=10)
    servo_2_pub_ = rospy.Publisher('/servo_2_position', Int16, queue_size=10)
    servo_3_pub_ = rospy.Publisher('/servo_3_position', Int16, queue_size=10)
    rospy.loginfo("ROS publisher on /servo_1_position [std_msgs/Int16]")
    rospy.loginfo("ROS publisher on /servo_2_position [std_msgs/Int16]")
    rospy.loginfo("ROS publisher on /servo_3_position [std_msgs/Int16]")
    
    while not rospy.is_shutdown():
        servo_1_.data = 100
        servo_1_pub_.publish(servo_1_)
        time.sleep(1)
        servo_2_.data = 90
        servo_2_pub_.publish(servo_2_)
        time.sleep(0.5)
        servo_3_.data = 20
        servo_3_pub_.publish(servo_3_)
        time.sleep(2)
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass