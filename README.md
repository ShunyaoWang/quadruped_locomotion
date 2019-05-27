# quadruped_locomoton
package for quadruped locomotion

## Dependencies
- [kindr](https://github.com/ANYbotics/kindr)
- [grid_map](https://github.com/ANYbotics/grid_map)
- [simple_dog_simulatuion](https://github.com/HITSZ-LeggedRobotics/simple_dog_simulation)
- [ooqp_eigen_interface](https://github.com/HITSZ-LeggedRobotics/dependencies/tree/master/ooqp_eigen_interface-master)
- [MA27](https://github.com/HITSZ-LeggedRobotics/ma27)
- [OOQP](https://github.com/HITSZ-LeggedRobotics/OOQP)
- [RBDL](https://github.com/HITSZ-LeggedRobotics/rbdl)
- [others](https://github.com/HITSZ-LeggedRobotics/dependencies)

## Build Dependencies
   - Build a floder named libraries to store these files;	
	
	mkdir libraries
	cd libraries 
	git clone git:..........git// download form the corredponding websites
   
   - first build and install **kindr**

  
   - build and install **RBDL** follow instructions in the corresponding floder;


    ```
    mkdir build
    cd build
    cmake -D CMAKE_BUILD_TYPE=Release RBDL_BUILD_ADDON_URDFREADER=TRUE ../
    make install
    ```
   then and .cmake file for cmake to find RBDL
   
    ```
    cd usr/local/lib/cmake
    sudo mkdir rbdl
    sudo cp RBDLConfig.cmake /usr/local/lib/cmake/rbdl
    ```  
  - git clone other ROS dependence package AND run `catkin_make`

## Install

## Usage
- ### Launch simulation environment
  `roslaunch simpledog simpledog_simulation.launch`
- ### Launch free_gait
  `roslaunch free_gait_ros test.launch`
- ### Start rqt user interface
  ` roslaunch balance_controller rqt_interface.launch `
After these step, there would be a gazebo simulation and a rqt interface.
  - First, move the dog model and place it in the gazebo.
  - Second, refresh the action collections, and send a sitdown action
  - push the **switch** botton, if the robot move correctly, then it start normally; otherwise, push **Back** botton, the robot reset to the ground
  - click **Trot** botton, the robot start torting, click **stop**, it stops troting, when torting, you can send cmd_vel msg to control the move.
![rqt_free_gait_interface](/assets/rqt_free_gait_interface.png)
