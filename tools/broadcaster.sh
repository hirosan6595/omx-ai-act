#!/bin/bash
# OMX-L(リーダー)の joint_trajectory_command_broadcaster を止める/動かす(ホストで実行)。
#   推論の前: ./broadcaster.sh off   (必須。動いているとグリッパーの指令がぶつかり、つかめない)
#   録画の前: ./broadcaster.sh on
# 注意: on にすると、フォロワーがリーダーの姿勢へ急に動く。録画前は omx_ai.launch を再起動する方が安全。
case "$1" in off) A=--deactivate ;; on) A=--activate ;; *) echo "usage: $0 off|on"; exit 1 ;; esac
docker exec open_manipulator bash -c "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_zenoh_cpp; \
  ros2 control switch_controllers -c /leader/controller_manager $A joint_trajectory_command_broadcaster && \
  ros2 control list_controllers -c /leader/controller_manager"
