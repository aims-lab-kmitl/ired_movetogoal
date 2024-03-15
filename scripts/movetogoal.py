#!/usr/bin/python3
"""
http://wiki.ros.org/move_base
http://wiki.ros.org/actionlib
"""
import rospy
import tf.transformations
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from std_msgs.msg import Empty, String, Int16
import actionlib

toggle_ = Empty()
position_ = String()
servo_ = Int16()

def pose_callback(pose_with_covariance):
    # print(pose_with_covariance)
    pose = pose_with_covariance.pose.pose
    print("amcl_pose = {x: %f, y:%f, orientation.z:%f" % (pose.position.x, pose.position.y, pose.orientation.z))


def move_base_status_callback(status):
    pass


def move_base_result_callback(result):
    pass


class moveBaseAction():
    def __init__(self):
        self.move_base_action = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_action.wait_for_server(rospy.Duration(5))

    def createGoal(self, x, y, theta):
        # quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(quat[0], quat[1], quat[2], quat[3]))

        return goal

    def moveToPoint(self, x, y, theta):
        target_point = self.createGoal(x, y, theta)
        self.moveToGoal(target_point)

    def moveToGoal(self, goal):
        self.move_base_action.send_goal(goal)
        success = self.move_base_action.wait_for_result()
        state = self.move_base_action.get_state()
        print ("Move to %f, %f, %f ->" % (
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y,
            goal.target_pose.pose.orientation.z
        ))
        if success and state == GoalStatus.SUCCEEDED:
            print(" Complete")
            return True
        else:
            print(" Fail")
            self.move_base_action.cancel_goal()
            return False


# Main program
def main():
    rospy.init_node('ired_movetogoal_node', anonymous=True)
    publisher_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    toggle_led_ = rospy.Publisher('/toggle_led', Empty, queue_size=10)
    position_pub_ = rospy.Publisher('/position_status', String, queue_size=10)
    servo_pub_ = rospy.Publisher('/servo_position', Int16, queue_size=10)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)

    # TODO
    mba = moveBaseAction()
    while not rospy.is_shutdown():
        mba.moveToPoint(3.471, -1.897, 0.0)
        position_.data = "Position 1"
        toggle_led_.publish(toggle_)
        position_pub_.publish(position_)
        servo_.data = 10
        servo_pub_.publish(servo_)
        time.sleep(3)
        mba.moveToPoint(3.726, 0.160, 3.14)
        toggle_led_.publish(toggle_)
        position_.data = "Position 2"
        toggle_led_.publish(toggle_)
        position_pub_.publish(position_)
        time.sleep(3)
        mba.moveToPoint(0.767, 0.527, -1.57)
        toggle_led_.publish(toggle_)
        position_.data = "Position 3"
        toggle_led_.publish(toggle_)
        position_pub_.publish(position_)
        time.sleep(3)

        rospy.sleep(1)

    rospy.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass