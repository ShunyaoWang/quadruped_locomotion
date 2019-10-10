# quadruped_locomoton
this is the branch EdisonKun_dev
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
   
   - first build and install **kindr**

  
   - build and install **RBDL** follow instructions in the corresponding floder;


    ```
    mkdir build
    cd build
    cmake -D CMAKE_BUILD_TYPE=Release RBDL_BUILD_ADDON_URDFREADER=TRUE ../
    sudo make install
    ```
   then and .cmake file for cmake to find RBDL
   
    ```
    cd usr/local/lib/cmake
    sudo mkdir rbdl
    sudo cp RBDLConfig.cmake /usr/local/lib/cmake/rbdl
    ```
  - git clone other ROS dependence package AND run `catkin_make`
  - **Bugs**
    - first build package >>free_gait_msgs
    - then build package sim_assiants in simpledog_simulation
## Install

## Usage
- ### Launch simulation environment
  `roslaunch simpledog quadruped_simulation.launch`
- ### Launch real robot(not fully surpported but can use for motor test)
    `roslaunch balance_controller balance_conoller_manager.launch `
- ### Launch free_gait
  `roslaunch free_gait_ros test.launch`
- ### Start rqt user interface
  ` roslaunch balance_controller rqt_interface.launch `

After these step, there would be a gazebo simulation and a rqt interface.
  - First, move the dog model and place it in the gazebo.
  - Second, refresh the action collections, and send a sitdown action
  - push the **switch** botton, if the robot move correctly, then it start normally; otherwise, push **Back** botton, the robot reset to the ground
  - click **Trot** botton, the robot start torting, click **stop**, it stops troting, when torting, you can send cmd_vel msg to control the move.
  - control panel in the right bottom can use to switch controller and controll single joint.
![rqt_free_gait_interface](/assets/rqt_free_gait_interface.png)
- ### Use Interactive Marker
 If use a interactive marker in RVIZ to send a single footstep goal;
 instead launch the **rqt_interface.launch** , do:
 `roslaunch simpledog quadruped_interactive_marker.launch`

### Control Panel
when you launch the file for real robot, you actually connected to some EtherCAT driver, so you can use control panel to control motor via EtherCAT.
![control_panel](/assets/control_panel.png)
