# 2D Simulation of IGVC

This repository contains models, worlds, plugins, and configuration files for the Flatland simulator to simulate IGVC.

## Installation

To use igvc_flatland, simply clone this repository and the Flatland repository into a ROS workspace of your choice. It is recommended to use the following Flatland repository: [https://github.com/robustify/flatland](https://github.com/robustify/flatland)

which is a fork of the main Flatland repository maintained by Avidbots: [https://github.com/avidbots/flatland](https://github.com/avidbots/flatland)

After cloning both igvc_flatland and Robustify's fork of Flatland, run catkin_make to compile everything.

## Launch the Simulator

There are two simulation environments in igvc_flatland: a basic course and an advanced course. There are four launch files in the `launch` folder to start the simulation:

-  `basic_course_north.launch`
-  `basic_course_south.launch`
-  `advanced_course_north.launch`
-  `advanced_course_south.launch`

Each one loads the specified world and spawns the robot facing either North or South, just as in the competition.

## Flatland Itself
To learn more about Flatland in general, check out the documentation page [http://flatland-simulator.readthedocs.io/en/latest/](http://flatland-simulator.readthedocs.io/en/latest/), as well as the source code. **In the specialized viewer window that opens when Flatland is loaded, the robot can be moved and rotated with the mouse if the `Interact` button on the toolbar at the top of the screen is clicked.** This feature is not currently described on the official Flatland documentation.

## Interacting with the Simulation
After running one of the above launch files, the following topics become available for use by the navigation software:

- **`/cmd_vel` (subscribe)**
	`geometry_msgs/Twist` topic that should be published to command the velocity of the robot.
- **`/twist` (publish)**
	`geometry_msgs/TwistStamped` topic containing the velocity feedback from the vehicle, which imitates the data that would come from wheel encoders.
- **`/gps/fix` (publish)**
	`sensor_msgs/NavSatFix` topic containing the perfectly-accurate GPS position of the robot.
- **`/waypoints` (publish)**
	`igvc_flatland/WaypointList` topic containing the list of waypoints for the particular course.
- **`/waypoint_markers` (publish)**
	`visualization_msgs/MarkerArray` topic to visualize the waypoints and their names and radii in Rviz.
- **`/scan` (publish)**
	`sensor_msgs/LaserScan` topic containing the scan data from a simulated LIDAR that returns data from the physical obstacles on the course.
- **`/camera_scan` (publish)**
	`sensor_msgs/LaserScan` topic containing the ideal output from a vision system, which in this case is another LIDAR scan that returns data from the lines.

## Automatic Scoring Simulation
An automatic judging plugin is run when the simulation is started. This plugin is implemented in `AutoJudge.cpp` found in the `src` folder. AutoJudge does the following:

- Times the entire run, and stops the run when the time limit is reached
- Monitors waypoint distances and counts the number of hit waypoints
- Checks for obstacle collisions and lane departures, which end the run
- Checks for reckless driving penalties, where the robot partially crosses a line
- Detects if the robot hasn't moved a sufficient forward progress in 1 minute, in which case the run is terminated
- Outputs final score to the terminal, depending on the circumstances of the run's termination, consistent with real IGVC rules 

AutoJudge starts its routine when it detects the first message published to the vehicle command topic `/cmd_vel`. This way, the simulator can be brought up before the main IGVC system code without the timer starting to run.

## Plugin YAML Parameters
Will document eventually...
However, changing the YAML parameters of the various Flatland plugins is only necessary if making modifications to the simulation itself. The information in the above sections should suffice to use the simulator to do preliminary testing of an IGVC system.