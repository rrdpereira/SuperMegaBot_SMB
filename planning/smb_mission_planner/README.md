## General Info
This repo adds mission planning functionality to the stack.
Because it should be easy to get into the code and modify it, everything is kept compact and at a minimum level of abstraction.
The mission planner is based on the popular state machine library `smach`, but no prior knowledge is required.

Some terminology to start with:
- In the context of this package, a waypoint denotes a named pose in the global frame.
- A mission is a collection of tasks which are executed sequentially (e.g. a path through several waypoints).
- A mission plan defines the logic how and when different missions are executed. The mission plan is implemented as a `smach` state machine.
- Each mission has its own mission data, containing relevant information for the mission (e.g. its waypoints).

## Installation
- Clone this repo into your workspace. _If you followed the instructions on how to install the SMB software, the source code of this package is already located in your workspace._
- Install `smach_ros` from [here](http://wiki.ros.org/smach_ros) (a state machine library written in python with ROS support)
- Install `yaml` with `pip install pyyaml` 
- (Buid the package with `catkin build smb_mission_planner`)



#### Constructing a State Machine
A mission plan (state machine) defines the connections (transitions) between different states. 
The mission plan is implemented as a `smach` state machine.

The proposed workflow for planning a navigation mission could be as follows:

1. Record mission data by using the `mission_recorder`
2. Create a class with your plan in the folder `src/smb_mission_planner/missions`
3. Execute your missions by using the `mission_planner`

Each of these steps is explained in detail in the upcoming sections.


## Record missions

### Basic Features
As it is fairly tedious to input poses manually for the mission waypoints, the `mission_recorder` helps you out.
It generates a `yaml` file with all the waypoint poses you recorded, grouped by mission.

You can launch it with
```
roslaunch smb_mission_planner mission_recorder.launch
```
which starts the node.
You can then give recording instructions with ros services.
To record a mission, call
```
rosservice call /record_mission {"mission_name","waypoint_1_name, waypoint_2_name, ..."}
```
where you can use your own `mission_name` and `waypoint_names`.
The number of waypoints can be selected arbitrarily, just add more to the list.
After you sent the `/record_mission` service, instructions will appear in the command window where you launched the node.
You can now input the waypoint poses of the current mission one by one.
This can be done

- in `rviz` by clicking `2D Nav Goal` and visually placing the pose on your map.
- by sending the desired pose in the topic `/move_base_simple/goal`.
- in `rviz` by sending a goal with the `smb_path_planner` widget.
- by calling the service `rosservice call /record_base_pose`, which will record the current base pose as a waypoint. Make sure that the odometry topic for the base pose is set correctly (see *Advanced Features* on how to do that).

After having recorded all your missions, stop the node with `Ctrl-C`.
All your recorded missions will be dumped to a `yaml` file (`mission.yaml` per default).
Of course, you can also *manually edit* the generated `yaml` file to combine different recording sessions and to add or edit waypoints manually, etc.


### Advanced Features

#### Remove Missions
Remove missions while the node is running with:
```
rosservice call /remove_mission "mission_name"
```

#### Remove Waypoint
Remove waypoints in missions while the node is running with:
```
rosservice call /remove_waypoint {"mission_name","waypoint_name"}
```

#### Specify file for file dump
You can use a `roslaunch` argument to specify a filepath for the output file, e.g.
```
roslaunch smb_mission_planner mission_recorder.launch mission_file_name:=mission_name.yaml
```

#### Prevent file dump
Prevent file dump with
```
rosservice call /toggle_file_dump "False"
```
or re-enable it with "True".


#### Choose your own input topic for recording
You can use a `roslaunch` argument to specify the waypoints' pose topic for the recording:
```
roslaunch smb_mission_planner mission_recorder.launch waypoint_topic_name:=/move_base_simple/goal
```

#### Choose your own odometry topic for recording
You can use a `roslaunch` argument to specify the base's frame and the reference frame:
```
roslaunch smb_mission_planner mission_recorder.launch base_frame:=base_link reference_frame:=map
```


## Mission planning

### Basic Features
You can combine your recorded missions to create a mission plan by connecting them to each other in the `mission_planner.py`.
The `mission_planner.py` should be modified by you to add more missions and connect them accordingly.
Here, a `smach` state machine is built up.
To learn more about it, visit the [tutorials](http://wiki.ros.org/smach/Tutorials).
Make sure to assign to each mission its respective mission data, i.e. its recorded information of the `yaml` file.

Currently, we provide you with a `TwistMission` and `WaypointMission` (see the files in `src/smb_mission_planner/mission`), which implements the following:

- Waypoints are set one by one, in the order they were defined in each mission in the `yaml` config file.
- If the robot is unable to reach a waypoint, it will abort the mission.
- A waypoint is reached if the xy-position and the yaw-angle are within a certain tolerance.
- If it cannot find the start of a mission, it will abort it.

To add a new mission type with your custom behaviour, see the next subsection below.

### Advanced features
Add your own mission types (e.g. to trigger a measurement instead of just reaching a waypoint):
- Create a new mission class (similar to the `WaypointMission` in a new file).
- Don't forget to inherit from `smach.State` and to implement the `__init__` and `execute` methods.
- Add your new mission type to the `mission_planner.py` to use it.


## Executing your mission plan

### Basic features
The `mission_planner.py` executes the previously defined mission plan.

Start the simulation and the path planner.
As soon as the path planner is ready to receive goals, start the `mission_planner` with
```
roslaunch smb_mission_planner mission_planner.launch
```
The robot will now try to reach the specified waypoints of each mission one by one, as defined in the `yaml` file.
In the command line window of the `mission_planner` you can find information where and in which mission you currently are in.

### Advanced features

#### Specify mission data file
You can use a `roslaunch` argument to specify a filepath for the input file, e.g.
```
roslaunch smb_mission_planner mission_planner.launch mission_file_name:=mission_name.yaml
```

#### Choose your own goal topic
You can use a `roslaunch` argument to specify the waypoints' pose topic:
```
roslaunch smb_mission_planner mission_planner.launch waypoint_topic_name:=/move_base_simple/goal
```

#### Choose your own odometry topic
You can use a `roslaunch` argument to specify the base's pose topic:
```
roslaunch smb_mission_planner mission_planner.launch base_frame:=base_link reference_frame:=map
```


## Where to go from here
- Try to record and execute a mission plan
- Modify the mission plan in the `mission_planner.py` file, by adding e.g. more mission states of the `WaypointMission` to the state machine.
- Add your own mission types, e.g. to trigger a measurement instead of just reaching a waypoint.


## Common pitfalls
- It is easy to forget to change the mission names in the `mission_planner.py` when recording new missions.
- The rosservice argument must be of the form `{"mission_name","waypoint_1_name, waypoint_2_name, ..."}` without a space after the separating comma between the strings.
- Make sure that your waypoints' pose topic, your base pose frame and reference frame are set correctly. 
