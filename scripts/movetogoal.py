#!/usr/bin/env python3

import rospy
import tf2_ros
import tf.transformations
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int16
from ired_movetogoal.srv import movetogoal, movetogoalResponse
from std_srvs.srv import Empty, EmptyResponse
import actionlib

class moveBaseAction():
    def __init__(self):
        self.navi_state_pub_ = rospy.Publisher("/movetogoal/status", Int16, queue_size=10)
        rospy.Service('/movetogoal/pose', movetogoal, self.movetoGoal)
        rospy.Service("/movetogoal/stop", Empty, self.movetoStop)
        
        self.move_base_action = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Wait for move_base")
        self.move_base_action.wait_for_server(rospy.Duration(5))
        
        self.tfBuffer = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.move_to_x_ = None
        self.move_to_y_ = None
        self.move_to_theta_ = None
        self.navigating = Int16()
        self.navigating.data = 2
    
    def createGoal(self):
        quat = tf.transformations.quaternion_from_euler(0, 0, self.move_to_theta_)
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = Pose(Point(self.move_to_x_, self.move_to_y_, 0.000), Quaternion(quat[0], quat[1], quat[2], quat[3]))

        return self.goal
    
    def movetoGoal(self, req):
        self.move_to_x_ = float(req.x)
        self.move_to_y_ = float(req.y)
        self.move_to_theta_ = float(req.theta)
        rospy.loginfo(f"Goto X: {self.move_to_x_} Y: {self.move_to_y_} Theta: {self.move_to_theta_}")
        
        target_point = self.createGoal()
        self.move_base_action.send_goal(target_point)
        
        return movetogoalResponse(True)
    
    def movetoStop(self, req):
        self.move_base_action.cancel_all_goals()
        
        return EmptyResponse()
    
    def getCurrentPose(self):
        try:
            trans_ = self.tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
            x_ = float(trans_.transform.translation.x)
            y_ = float(trans_.transform.translation.y)
            tf_angle_ = tf.transformations.euler_from_quaternion([trans_.transform.rotation.x, trans_.transform.rotation.y, trans_.transform.rotation.z, trans_.transform.rotation.w])
            move_to_theta_ = float(tf_angle_[2])
            
            return (x_, y_, move_to_theta_)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Wait for Transform")
            return (-1.0, -1.0, -1.0)
    
    def checkNavigationStatus(self):
        current_pose_ = self.getCurrentPose()
        if current_pose_[0] == -1 or current_pose_[1] == -1 or current_pose_[2] == -1:
            return
        if self.move_to_theta_ is None or self.move_to_x_ is None or self.move_to_y_ is None:
            return
        state_ = self.move_base_action.get_state()
        abs_goal_x_ = abs(current_pose_[0] - self.move_to_x_)
        abs_goal_y_ = abs(current_pose_[1] - self.move_to_y_)
        abs_goal_theta_ = abs(current_pose_[2] - self.move_to_theta_)
        if abs_goal_x_ < 0.1 and abs_goal_y_ < 0.1 and abs_goal_theta_ < 0.174532925 or state_ == GoalStatus.SUCCEEDED:
            self.move_base_action.cancel_all_goals()
            self.navigating.data = 1 # SUCCEEDED
        elif state_ == GoalStatus.ACTIVE or state_ == GoalStatus.PENDING:
            self.navigating.data = 0 # Processing
            # rospy.loginfo(f"X: {current_pose_[0]} Y: {current_pose_[1]} Theta: {current_pose_[2]}")
        else:
            self.navigating.data = 2 # ABORTED
        
        return True
            
    def publishNavigationState(self):
        self.navi_state_pub_.publish(self.navigating)
        
        return True

def main():
    rospy.init_node("ired_movetogoal_node")
    rate = rospy.Rate(10)
    move2goal = moveBaseAction()
    
    while not rospy.is_shutdown():
        move2goal.checkNavigationStatus()
        move2goal.publishNavigationState()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass