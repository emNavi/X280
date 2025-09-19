# X280 REV

# 自动化三维建图无人机使用文档

![EGC](./EGC.png)

主要组件说明：
- 雷达：Mid 360 10Hz
- 相机：海康威视工业相机 10Hz 1440x1080 RGB
- IMU：飞控 200Hz
- RTK：10Hz 串口通讯 /dev/ttyACM1
- 自组网系统：平均带宽 2~4 MB/s

开始之前请确保无人机与主机位于自组网内能互相访问
- 主机IP设置为 192.168.1.100
- 无人机IP设置为 192.168.1.101
- 机器狗IP设置为 192.168.1.18

## 项目编译
```bash
cd EGC
catkin_make
# 新开启一个终端更新项目环境变量（永久写入 ~/.bashrc 也可以）
source devel/setup.bash

# 为无人机配置 ssh 免密访问
# 在主机执行，连续按三下回车，就结束了
ssh-keygen -t rsa
# 然后同步秘钥到无人机，此时输入无人机密码 123456
ssh-copy-id emnavi@192.168.1.101
# 至此以后该主机访问无人机不再需要密码
```

## 演示开始
```bash
# 开启一个终端，打开 EGC 界面
roslaunch egc gui.launch
```

```bash
# 再开启一个终端，远程连接至无人机终端并监听主机消息
ssh emnavi@192.168.1.101

bash ~/X280/scripts/run_remote.sh
```


## 建图说明
待无人机放稳后点击 `启动无人机建图` 按钮，等待片刻，当rviz地图上有无人机坐标及彩色点云显示时，即表明无人机已就绪。

## 设置静态外参说明
| 坐标系名称                          | 说明                     | 
|---------------------------------|------------------------------|
|  global_world    | 全局世界坐标系，联合建图都在上面可视化     | 
|  uav_world     | 无人机定位的世界坐标系               | 
|  base_view     | 机器狗定位的世界坐标系                |


机器狗激光里程计启动时，以当前位置机器狗坐标系（base_view）原点,并同时设置全局世界坐标系（global_world）也是这个点。
这里我们假设无人机定位里程计启动时，会有一个名为 uav_world 的无人机世界坐标系。为了使得无人机的建图也能正确显示在全局世界坐标系（global_world）下
此时需要标定出 uav_world 到 base_view 的静态外参，这就等价于直接得到了 uav_world 到 global_world 的静态外参。


在QT界面中，静态外参输入格式应为：
x y z qx qy qz qw

例如：0.03 0.27 0.06 0 0 0 1

按下回车后，会自动将名为 uav_world 的坐标系变换到 global_world 上


## 录制 ROS1 bag 包
先点击 `采集数据 Bag 包` 按钮，等待采集完成之后，点击 `停止数据采集`。录制的 bag 包会默认放在主机的 EGC 项目目录下，名称为录制的时间。默认录制的话题如下：

| 话题名                          | 消息类型                     | 备注                         |
|---------------------------------|------------------------------|------------------------------|
| `/hik_camera_1/camera_info`     | `sensor_msgs/CameraInfo`     | 海康相机的内参、畸变参数等相机标定信息 |
| `/hik_camera_1/image/compressed`| `sensor_msgs/CompressedImage`| 海康相机的压缩图像流（JPEG） |
| `/livox/lidar`                  | `livox_ros_driver2/CustomMsg`| Livox 激光雷达原始点云数据        |
| `/mavros/imu/data_raw`          | `sensor_msgs/Imu`            | 飞控原始 IMU 数据（加速度计 + 陀螺仪） |
| `/mavros/gps/fix`               | `sensor_msgs/NavSatFix`      | 飞控 GPS 定位信息（经纬度、高度） |

## 起飞 & 降落
点击 `起飞` 按钮，稍等片刻，无人机开始起飞至预设悬停高度 （默认0.5米）。
演示过程中可随时点击  `降落` 按钮，可使无人机立即降落。


## 无人机规划说明
无人机规划默认接收 /uav_goal 话题，选用地图指点的方式进行操作，先点击 `运行规划导航算法` 按钮，等待 2 秒，点击 `2D 导航目标点`, 然后在 rviz 界面中使用鼠标左键点击进行标点。 规划成功后，可在 rviz 中看到红色的规划轨迹。

## 演示结束
关闭 EGC 软件界面，并退出终端进程。无人机端可自动结束已运行的ROS服务。


## （手动调试用）在无人机中使用
```bash
# 1、启动飞控、传感器、激光里程计算法
bash scripts/init_uav.sh

# 2、录制 bag 包
rosbag record /hik_camera_1/camera_info /hik_camera_1/image/compressed /livox/lidar /mavros/imu/data_raw /mavros/gps/fix

# 3、起飞
bash scripts/take_off.sh

# 4、启动无人机导航规划算法
bash scripts/run_egoplanner.sh

# 5、指点飞行

# 6、降落
bash scripts/land.sh
```

## 其他说明
1、无人机电池记得随时检查电压，低于 18V 请充电后再进行飞行。

2、飞行过程中请飞手全程手持遥控器，防止意外便于接管。

3、一段时间静止，无人机可能会发出滴滴响声，这是无人机防止丢失报警，此时用遥控器简单拨 解锁-上锁 就能解除。

4、户外条件下，无人机自动接收 RTK 信号，从第一次收到信号后，断连后就会发出持续30秒蜂鸣器音，直到能再次收到信号。

5、无人机完全降落至地面时，油门会先变大然后再自动停桨，属于正常现象，无需额外操作。

