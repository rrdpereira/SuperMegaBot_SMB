#!/usr/bin/env python

import rospy
import yaml
import smach_ros
import smach
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from smb_mission_planner.missions.waypoint_mission import WaypointMission
from smb_mission_planner.missions.twist_mission import TwistMission

class MissionPlan():
    def __init__(self, missions_data, reference_frame):
        self.missions_data = missions_data
        self.reference_frame = reference_frame

    def createStateMachine(self):
        state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])
        with state_machine:
            smach.StateMachine.add('Waypoint Mission', WaypointMission(self.missions_data['waypoint_mission'], self.reference_frame),
                                   transitions={'Completed': 'Twist Mission', 'Aborted': 'Failure', 'Next Waypoint': 'Waypoint Mission'})
            smach.StateMachine.add('Twist Mission', TwistMission(self.missions_data['twist_mission'], self.reference_frame),
                                   transitions={'Completed': 'Success', 'Aborted': 'Failure', 'Next Twist': 'Twist Mission'})
        return state_machine

class MissionPlanner():
    def __init__(self, yaml_file_path, reference_frame):
        # Read missions data.
        self.yaml_file_path = yaml_file_path
        self.reference_frame = reference_frame
        self.readMissionsData()

        self.main()

    def readMissionsData(self):
        with open(self.yaml_file_path, 'r') as file:
            self.missions_data = yaml.load(file, Loader=yaml.FullLoader)

    def main(self):
        rospy.init_node('mission_planner_node')
        rospy.loginfo("Mission planner started.")

        # Setup state machine.
        mission_plan = MissionPlan(self.missions_data, self.reference_frame)
        state_machine = mission_plan.createStateMachine()

        # Create and start the introspection server.
        introspection_server = smach_ros.IntrospectionServer('mission_planner_introspection_server', state_machine, '/mission_planner')
        introspection_server.start()

        # Execute state machine.
        outcome = state_machine.execute()
        rospy.loginfo("Mission plan terminated with outcome '" + outcome + "'.")

        # Wait for ctrl-c to stop the application
        introspection_server.stop()
