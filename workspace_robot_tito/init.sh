#!/bin/bash

echo "iniciando robot's en ROS2"

cd tito_hardware/install

source  setup.bash

## inicio los robots definidos por udrf y la configuracion del controllers
ros2 run controller_manager ros2_control_node --ros-args --params-file  /tito_bringup/config/controllers.yaml -r /diff_drive_controller/cmd_vel:=/cmd_vel &

sleep 3

ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /tito_description/urdf/robot.urdf.xacro)" &

sleep 5


## inicio los controladores joint_state y diff_drive
ros2 run controller_manager spawner joint_state_broadcaster  &
 
ros2 run controller_manager spawner diff_drive_controller &

## inicio el agente ros2


cd micro_ros_agent/uros_ws/install || exit 1


source setup.bash

cd micro_ros_agent/uros_ws


ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888  




