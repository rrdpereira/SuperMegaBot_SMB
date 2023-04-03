#!/usr/bin/env python

import rospy
import collections
import yaml
import tf
from geometry_msgs.msg import PoseStamped
from smb_mission_planner.srv import RecordMission, RecordMissionResponse
from smb_mission_planner.srv import RemoveMission, RemoveMissionResponse
from smb_mission_planner.srv import RemoveWaypoint, RemoveWaypointResponse
from smb_mission_planner.srv import ToggleFileDump, ToggleFileDumpResponse
from smb_mission_planner.srv import RecordBasePose, RecordBasePoseResponse
from collections import OrderedDict
from tf.transformations import euler_from_quaternion
import tf2_ros

def ordered_dict_representer(self, value):  # can be a lambda if that's what you prefer
    return self.represent_mapping('tag:yaml.org,2002:map', value.items())
yaml.add_representer(OrderedDict, ordered_dict_representer)
class MissionRecorder():
    def __init__(self, yaml_file_path, waypoint_topic_name, reference_frame, base_frame):
        self.yaml_file_path = yaml_file_path
        self.waypoint_topic_name = waypoint_topic_name
        self.reference_frame = reference_frame
        self.base_frame = base_frame

        self.current_mission_name = ""
        self.current_waypoint_list = []
        self.waypoint_counter = 0
        self.missions_data = collections.OrderedDict()
        self.file_dump_on = True

        self.waypoint_pose_subscriber = rospy.Subscriber(self.waypoint_topic_name, PoseStamped, self.waypointCallback)

        self.record_mission_service = rospy.Service('record_mission', RecordMission, self.recordMission)
        self.remove_mission_service = rospy.Service('remove_mission', RemoveMission, self.removeMission)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.removeWaypoint)
        self.record_base_pose_service = rospy.Service('record_base_pose', RecordBasePose, self.recordBasePose)
        self.toggle_file_dump_service = rospy.Service('toggle_file_dump', ToggleFileDump, self.toggleFileDump)

        self.main()

    def recordMission(self, data):
        if(data.mission_name == ""):
            rospy.logwarn("The mission name cannot be empty. Recording of mission cancelled.")
            return RecordMissionResponse()
        self.current_mission_name = data.mission_name

        current_waypoint_list = list(map(str.strip, data.waypoint_names.split(',')))
        for waypoint_name in current_waypoint_list:
            if(waypoint_name == ""):
                rospy.logwarn("Waypoint names cannot be empty. Recording of mission cancelled.")
                return RecordMissionResponse()
        self.current_waypoint_list = current_waypoint_list

        self.waypoint_counter = 0
        rospy.loginfo("Recording of mission '" + self.current_mission_name + "' has started.")
        rospy.loginfo("Please record the pose of the waypoint '" + self.current_waypoint_list[0] + "'.")
        return RecordMissionResponse()

    def removeMission(self, data):
        if(data.mission_name == ""):
            rospy.logwarn("The mission name cannot be empty. Deleting of mission cancWelled.")
            return RemoveMissionResponse()
        try:
            del self.missions_data[data.mission_name]
            rospy.loginfo("Mission '" + self.current_mission_name + "' removed.")
            return RemoveMissionResponse()
        except:
            rospy.logwarn("The mission '" + data.mission_name + "'does not exist. Deleting of mission cancelled.")
            return RemoveMissionResponse()

    def waypointCallback(self, pose_stamped_msg):
        if(self.waypoint_counter < len(self.current_waypoint_list)):
            self.addWaypoint(self.current_mission_name, self.current_waypoint_list[self.waypoint_counter], pose_stamped_msg)
            rospy.loginfo("The pose of the waypoint '" + self.current_waypoint_list[self.waypoint_counter] + "' has been succesfully recorded.")
            self.waypoint_counter += 1
            if(self.waypoint_counter == len(self.current_waypoint_list)):
                rospy.loginfo("Recording of mission '" + self.current_mission_name + "' has been successfully completed.")
            else:
                rospy.loginfo("Please record the pose of the next waypoint '" + self.current_waypoint_list[self.waypoint_counter] + "'.")

    def addWaypoint(self, mission_name, waypoint_name, pose_stamped_msg):
        x_m = pose_stamped_msg.pose.position.x
        y_m = pose_stamped_msg.pose.position.y
        quaternion = pose_stamped_msg.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll_rad, pitch_rad, yaw_rad = tf.transformations.euler_from_quaternion(explicit_quat)

        new_waypoint = collections.OrderedDict({waypoint_name: {'x_m': x_m, 'y_m': y_m, 'yaw_rad': yaw_rad}})
        if(self.missions_data.get(mission_name) == None):
            self.missions_data.update({mission_name: new_waypoint})
        else:
            self.missions_data[mission_name].update(new_waypoint)

    def removeWaypoint(self, data):
        if(data.mission_name == ""):
            rospy.logwarn("The mission name cannot be empty. Deleting of waypoint cancelled.")
            return RemoveWaypointResponse()
        if(data.waypoint_name == ""):
            rospy.logwarn("The waypoint name cannot be empty. Deleting of waypoint cancelled.")
            return RemoveWaypointResponse()
        try:
            del self.missions_data[data.mission_name][data.waypoint_name]
            rospy.loginfo("Waypoint '" + data.waypoint_name + "' in mission '" + data.mission_name + "' removed.")
            return RemoveWaypointResponse()
        except:
            rospy.logwarn("The waypoint '" + data.mission_name + "/" + data.waypoint_name + "' does not exist.")
            return RemoveWaypointResponse()

    def recordBasePose(self, data):
        try:
            # Read base pose
            trans = self.tfBuffer.lookup_transform(self.reference_frame, self.base_frame, rospy.Time())
            smb_pose = PoseStamped()
            smb_pose.pose.position = trans.transform.translation
            smb_pose.pose.orientation = trans.transform.rotation
            # Save waypoint
            self.waypointCallback(smb_pose)
            return RecordBasePoseResponse()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Could not look up Transform")

    def toggleFileDump(self, data):
        self.file_dump_on = data.file_dump_on
        if(self.file_dump_on):
            rospy.loginfo("Dumping to file is currently on.")
        else:
            rospy.logwarn("Dumping to file is currently off.")
        return ToggleFileDumpResponse()

    def dump(self):
        if(self.file_dump_on):
            with open(self.yaml_file_path, 'w+') as file:
                yaml.dump(self.missions_data, file, default_flow_style=False)
                rospy.loginfo("Mission file succesfully dumped under: " + self.yaml_file_path)

    def main(self):
        rospy.init_node('mission_recorder_node')
        rospy.loginfo("Mission recorder ready.")
        rospy.loginfo("Waiting for '/record_mission' services.")

        # Initialize tf listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Wait for Ctrl-C to stop the application.
        rospy.spin()
        self.dump()
