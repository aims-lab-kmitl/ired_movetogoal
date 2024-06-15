#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from ired_movetogoal.srv import movetogoal, movetogoalResponse
from std_srvs.srv import Empty, EmptyResponse

navi_stat_ = Int16()
move2Pose_ = None
move2Stop_ = None

def navigationStatus(msg):
    global navi_stat_
    navi_stat_ = msg
    
def gotoPose(x:float, y:float, theta:float):
    global move2Pose_
    resp_ = move2Pose_(x, y, theta)
    return resp_.status

def main():
    global navi_stat_, move2Pose_, move2Stop_
    rospy.init_node('position_node')
    rospy.loginfo("Wait for /movetogoal/pose and /movetogoal/stop")
    rospy.wait_for_service('/movetogoal/pose')
    rospy.wait_for_service('/movetogoal/stop')
    rospy.loginfo("ROS Service Client Connected!")
    move2Pose_ = rospy.ServiceProxy('/movetogoal/pose', movetogoal)
    move2Stop_ = rospy.ServiceProxy('movetogoal/stop', Empty)
    rate = rospy.Rate(10)
    rospy.Subscriber("/movetogoal/status", Int16, navigationStatus)
    
    position_count_ = 0
    pose_ = [[0.602,1.748, 3.14], [-1.812, -0.587, 0.0]]
    gotoPose(pose_[position_count_][0], pose_[position_count_][1], pose_[position_count_][2])
    
    while not rospy.is_shutdown():
        # Processing
        if navi_stat_.data == 0:
            rospy.loginfo("Processing")
        
        # SUCCEEDED
        if navi_stat_.data == 1:
            position_count_ += 1
            if position_count_ > 2:
                break
            gotoPose(pose_[position_count_][0], pose_[position_count_][1], pose_[position_count_][2])
            rospy.loginfo("SUCCEEDED")
        
        # ABORTED
        if navi_stat_.data == 2:
            rospy.loginfo("ABORTED")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass