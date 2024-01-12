Introduction
-------------

This prototype shows how multiple robots can achieve a common mission while
keeping a formation.

Requirements
------------
 - Ubuntu 22.04
 - CoppeliaSim V 4.5
 - ROS2 Humble

Building
--------
  cd ros2_multirobots_ws
  source /opt/ros/humble/setup.bash 
  ./build_packages.sh
  source install/setup.bash

Mission example:
----------------
Move three E-Puck robots (named respectively ep1c, ep2c, and ep3c) from 
their current position to a given position while keeping their formation. 
The target position of the mass center of the three robots is specified.

Running the mission example
---------------------------

 - Launch the CoppeliaSim simulator and open the scene coppeliasim_files/epuck1.ttt 
   Do not start the simulation yet.

 - Launch the oracle in a first terminal
	ros2 run mr_oracle oracle

 - Launch the three controllers in a second terminal
	ros2 run epuck_controller controller ep1c 0.0 1.5  &
	ros2 run epuck_controller controller ep2c 0.0 1.5  &
	ros2 run epuck_controller controller ep3c 0.0 1.5  &

 - Launch the clock in a third terminal
	ros2 run mr_clock clock

 - Start the simulation in CoppeliaSim.
	
