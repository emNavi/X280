#!/bin/bash
echo "[START] Fast-LIO2 with X280 "
# 通过本脚本文件路径来获取 X280 项目文件根目录
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

if [ $? -eq 0 ] 
then
    echo "start mid360_ros && fast_lio"
    source export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH;
    source ${PROJECT_DIR}/devel/setup.bash;roslaunch fast_lio mapping_mid360.launch  && sleep 1;
    sleep 1;
else
    echo "[error]Not found drone_id,please_check params_file"
    exit 
fi
