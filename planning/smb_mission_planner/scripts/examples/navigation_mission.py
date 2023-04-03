#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from smb_mission_planner.navigation_states import WaypointNavigation
from smb_mission_planner.utils import ros_utils

"""
Example script consisting of performing different missions, each consisting of a navigation
through predefined waypoint which have been previously recorded
"""

rospy.init_node('navigation_mission_node')

# Parse params
mission_file = ros_utils.get_param_safe("~mission_file")
move_base_topic = ros_utils.get_param_safe("~move_base_topic")
odometry_topic = ros_utils.get_param_safe("~odometry_topic")

state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])
mission_data = WaypointNavigation.read_missions_data(mission_file)
with state_machine:
    smach.StateMachine.add('CHECK_FIRE_HAZARD', WaypointNavigation(mission_data['check_fire_hazard'],
                                                                   waypoint_pose_topic=move_base_topic,
                                                                   base_pose_topic=odometry_topic,
                                                                   ns="CHECK_FIRE_HAZARD"),
                           transitions={'Completed': 'GATHER_FRUITS',
                                        'Aborted': 'Failure',
                                        'Next Waypoint': 'CHECK_FIRE_HAZARD'})

    smach.StateMachine.add('GATHER_FRUITS', WaypointNavigation(mission_data['gather_fruits'],
                                                               waypoint_pose_topic=move_base_topic,
                                                               base_pose_topic=odometry_topic,
                                                               ns="GATHER_FRUITS"),
                           transitions={'Completed': 'Success',
                                        'Aborted': 'GATHER_VEGETABLES',
                                        'Next Waypoint': 'GATHER_FRUITS'})

    smach.StateMachine.add('GATHER_VEGETABLES', WaypointNavigation(modometry_topicission_data['gather_vegetables'],
                                                                   waypoint_pose_topic=move_base_topic,
                                                                   base_pose_topic=odometry_topic,
                                                                   ns="GATHER_VEGETABLES"),
                           transitions={'Completed': 'Success',
                                        'Aborted': 'Failure',
                                        'Next Waypoint': 'GATHER_VEGETABLES'})

# Create and start the introspection server
introspection_server = smach_ros.IntrospectionServer('mission_server', state_machine, '/mission_planner')
introspection_server.start()

# Execute state machine.
outcome = state_machine.execute()
rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))

# Wait for ctrl-c to stop the application
introspection_server.stop()
