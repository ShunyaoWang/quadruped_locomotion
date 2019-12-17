#!/bin/bash

# Setcap does not support symbolic links, so a potential symbolic link has to be resolved first.
resolved_symlink=$(readlink -f ${5})

# Setcap using password.
echo ${4} | sudo -S setcap cap_net_raw+ep ${resolved_symlink}

# Update the links and cache to the shared catkin libraries.
# See https://stackoverflow.com/questions/9843178/linux-capabilities-setcap-seems-to-disable-ld-library-path
sudo ldconfig /opt/ros/$ROS_DISTRO/lib

# Launch the node.
roslaunch roslaunch balance_controller balance_controller_manager.launch  output:="${1}" launch_prefix:="${2}"  time_step:="${3}"
