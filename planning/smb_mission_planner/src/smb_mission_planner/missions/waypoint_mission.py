#!/usr/bin/env python
import smach
import rospy
import tf
import smach
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class WaypointMission(smach.State):
    def __init__(self, mission_data, reference_frame):
        smach.State.__init__(
            self, outcomes=['Completed', 'Aborted', 'Next Waypoint'])

        # Get parameters
        # Frames
        self.reference_frame = reference_frame

        self.mission_data = mission_data
        self.waypoint_idx = 0
        self.waypoint_name = ""
        self.next_waypoint = False

        # actionlib client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            return

        rospy.loginfo("Waiting for goals..")

    def execute(self, userdata):
        if(self.waypoint_idx >= len(self.mission_data.keys())):
            rospy.loginfo("No more waypoints left in current mission.")
            self.waypoint_idx = 0
            return 'Completed'

        self.waypoint_name = list(self.mission_data.keys())[
            self.waypoint_idx]
        current_waypoint = self.mission_data[self.waypoint_name]

        self.setWaypoint(
            current_waypoint['x_m'], current_waypoint['y_m'], current_waypoint['yaw_rad'])
        rospy.loginfo("Waypoint set: '" + self.waypoint_name + "'.")

        self.client.wait_for_result()

        if self.next_waypoint:
            rospy.loginfo("Waypoint '" + self.waypoint_name +
                              "' reached. Loading next waypoint...")
            self.waypoint_idx += 1
            return 'Next Waypoint'
        else:
            rospy.logwarn(
                "Waypoint of mission unreachable. Aborting current mission.")
            self.waypoint_idx = 0.
            return 'Aborted'

    def active_cb(self):
        rospy.loginfo(self.waypoint_name +
                      " is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        # To print current pose at each feedback:
        #rospy.loginfo("Feedback for waypoint " + str(feedback))
        pass

    def done_cb(self, status, result):
        # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == GoalStatus.PREEMPTED:
            rospy.loginfo(self.waypoint_name+" received a cancel request after it started executing, successfully cancelled!")
            self.next_waypoint = False

        elif status == GoalStatus.RECALLED:
            rospy.loginfo(self.waypoint_name+" received a cancel request before it started executing, successfully cancelled!")
            self.next_waypoint = False

        elif status == GoalStatus.SUCCEEDED:
            rospy.loginfo(self.waypoint_name+" REACHED!") 
            self.next_waypoint = True

        elif status == GoalStatus.REJECTED:
            rospy.logerr(self.waypoint_name+" has been rejected by the Action Server. Stopping.")
            self.next_waypoint = True

        self._sending = False #ended dealing with the goal

    def setWaypoint(self, x_m, y_m, yaw_rad):
        quaternion = tf.transformations.quaternion_from_euler(0., 0., yaw_rad)

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.reference_frame
        goal.target_pose.pose.position.x = x_m
        goal.target_pose.pose.position.y = y_m
        goal.target_pose.pose.position.z = 0.
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
