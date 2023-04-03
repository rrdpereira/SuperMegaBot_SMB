#!/usr/bin/env python
import smach
import rospy
import tf
import smach
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class TwistMission(smach.State):
    def __init__(self, mission_data, reference_frame):
        smach.State.__init__(
            self, outcomes=['Completed', 'Aborted', 'Next Twist'])

        # Get parameters
        # Frames
        self.reference_frame = reference_frame

        self.mission_data = mission_data
        self.twist_idx = 0
        self.twist_name = ""
        self.next_twist = False

        self.update_rate = rospy.Rate(10)
        self.twist_pub = rospy.Publisher('/keyboard_teleop/cmd_vel', Twist, queue_size=1)
        

    def execute(self, userdata):
        if(self.twist_idx >= len(self.mission_data.keys())):
            rospy.loginfo("No more twists left in current mission.")
            self.twist_idx = 0
            return 'Completed'

        self.twist_name = list(self.mission_data.keys())[
            self.twist_idx]
        current_twist = self.mission_data[self.twist_name]

        self.setTwist(
            current_twist['lin_vel'], current_twist['ang_vel'], current_twist['time'])
        rospy.loginfo("Twist set: '" + self.twist_name + "'.")

        if self.next_twist:
            rospy.loginfo("Twist '" + self.twist_name +
                              "' published. Loading next twist...")
            self.twist_idx += 1
            return 'Next Twist'
        else:
            rospy.logwarn(
                "Can't publish twist. Aborting mission...")
            self.twist_idx = 0.
            return 'Aborted'

    def setTwist(self, lin_vel, ang_vel, time):
        twist_msg = Twist()
        twist_msg.linear.x = lin_vel
        twist_msg.angular.z = ang_vel
        endtime = rospy.Time.now() + rospy.Duration(time)

        while rospy.Time.now() < endtime:
            self.twist_pub.publish(twist_msg)
            self.update_rate.sleep()

        self.next_twist = True


