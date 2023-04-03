#!/usr/bin/env python

import argparse
from smb_mission_planner.mission_recorder import MissionRecorder


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Tool for recording missions.')
    parser.add_argument('config_file_path', type=str, help='Path of the config file.')
    parser.add_argument('waypoint_topic_name', type=str, help='Topic name for recording poses of waypoints.')
    parser.add_argument('reference_frame', type=str, help='Reference frame.')
    parser.add_argument('base_frame', type=str, help='Base frame.')

    # Ignore arguments sent by roslaunch.
    parser.add_argument('__name:', help=argparse.SUPPRESS, nargs='?')
    parser.add_argument('__log:', help=argparse.SUPPRESS, nargs='?')
    args = parser.parse_args()

    mission_recorder = MissionRecorder(args.config_file_path, args.waypoint_topic_name, args.reference_frame, args.base_frame)
