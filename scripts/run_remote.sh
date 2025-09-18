#!/bin/bash
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
source ${PROJECT_DIR}/scripts/common_func.sh

echo_G "[START] Init UAV Remote Control"
drone_type=$(python3 ${PROJECT_DIR}/scripts/find_config.py drone_type)
drone_id=$(python3 ${PROJECT_DIR}/scripts/find_config.py drone_id)

# 该脚本是用来监听主机端的指令，并转换为无人机的命令执行
source /home/emnavi/.bashrc
source /home/emnavi/X280/devel/setup.bash;roslaunch global_interface remote_script.launch drone_type:=$drone_type drone_id:=$drone_id && sleep 1;
sleep 1;

