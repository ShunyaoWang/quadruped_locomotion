# quadruped_locomoton
package for quadruped locomotion
## Dependencies
- [kindr](https://github.com/ANYbotics/kindr)
- [grid_map](https://github.com/ANYbotics/grid_map)
- [simple_dog_simulatuion](https://github.com/HITSZ-LeggedRobotics/simple_dog_simulation)
- [others](https://github.com/HITSZ-LeggedRobotics/dependencies)

## Build Dependencies
  - first install **kindr** and git clone other ROS dependence package AND run `catkin_make`

## Install

## Usage
- ### Launch simulation environment
  `roslaunch simpledog simpledog_simulation.launch`
  `roslaunch simpledog display.launch`
- ### Launch free_gait
  `roslaunch free_gait_ros test.launch`
- ### Start rqt user interface
  ` rqt --perspective-file (path to balance_controller)/config/free_gait_control.perspective `
After these step, there would be a gazebo simulation and a rqt interface.
  - First, move the dog model and place it in the gazebo.
  - Second, refresh the action collections, and send a sitdown action
  - push the **switch** botton, if the robot move correctly, then it start normally; otherwise, push **Back** botton, then put the robot again, and in the controller mannager, unload and load the balance_controller. repeat the fisrt three steps.
![rqt_free_gait_interface](/assets/rqt_free_gait_interface.png)
