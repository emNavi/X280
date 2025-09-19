#!/bin/bash
# 通过本脚本文件路径来获取 X280 项目文件根目录、飞机类型、飞机编号、起飞高度、gcs_udp_ip
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
source ${PROJECT_DIR}/scripts/common_func.sh

echo_G "[START] one_shot script with X280"
rosrun emnv_ctl_bridge reboot_px4_system.py && sleep 1;

nodes=(
/rosout
/x280_2/ctrl_bridge
/x280_2/drone_1_ego_planner_node
/x280_2/drone_1_traj_server
/x280_2/fastlio
/x280_2/livox_lidar_publisher
/x280_2/local_map
/x280_2/mavros
/x280_2/racer_topic
/x280_2/data_manager
)

for n in "${nodes[@]}"; do
    echo "Killing $n ..."
    rosnode kill "$n"
done