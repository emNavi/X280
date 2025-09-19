#include <ros/ros.h>
#include <ros/master.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <limits>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <Eigen/StdVector>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <signal.h>

#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include <geometry_msgs/Pose.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <iostream>
#include <string>
#include <cstdlib>

// 全局外参变量
tf2_ros::Buffer tf_buffer;
using namespace Eigen;
Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();

ros::Subscriber odom_sub_racer, sub_uav_odom, sub_uav_path, sub_uav_pc;
ros::Publisher pub_odom_mavros, pub_odom_racer, pub_pose_racer, pub_global_world_odom, pub_global_world_path, pub_global_world_pc;
std::string odom_racer_id= "1" ;
float racer_x = 0, racer_y = 0, racer_z = 0;
bool odom_enable=false;

// 用于检查话题是否存在
bool topicExists(const std::string& topic_name)
{
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    for (auto& t : master_topics) {
        if (t.name == topic_name) return true;
    }
    return false;
}

// 将激光坐标系下的里程计转为世界坐标系下，并发布给 mavros 的 vision_pose 话题
void odomToUavWorldCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    // 从里程计获取位姿
    Eigen::Quaterniond q(
        odom_msg->pose.pose.orientation.w,
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z
    );
    Eigen::Matrix3d rot = q.toRotationMatrix();
    Eigen::Vector3d trans(
        odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z
    );

    // 构造4x4变换矩阵
    Eigen::Matrix4d odom_transform = Eigen::Matrix4d::Identity();
    odom_transform.block<3,3>(0,0) = rot;
    odom_transform.block<3,1>(0,3) = trans;

    // 应用外参变换
    Eigen::Matrix4d odom_in_uav_world = extrinsic * odom_transform * extrinsic.inverse();

    // 提取变换后的旋转和平移
    Eigen::Matrix3d rot_new = odom_in_uav_world.block<3,3>(0,0);
    Eigen::Vector3d trans_new = odom_in_uav_world.block<3,1>(0,3);
    Eigen::Quaterniond q_new(rot_new);

    // 发布转换后的里程计信息
    nav_msgs::Odometry odom_out = *odom_msg;
    odom_out.header.frame_id = "uav_world";
    odom_out.pose.pose.position.x = trans_new.x();
    odom_out.pose.pose.position.y = trans_new.y();
    odom_out.pose.pose.position.z = trans_new.z();
    odom_out.pose.pose.orientation.x = q_new.x();
    odom_out.pose.pose.orientation.y = q_new.y();
    odom_out.pose.pose.orientation.z = q_new.z();
    odom_out.pose.pose.orientation.w = q_new.w();


    // === 新增 Twist 速度转换 ===
    Eigen::Vector3d v_old(
        odom_msg->twist.twist.linear.x,
        odom_msg->twist.twist.linear.y,
        odom_msg->twist.twist.linear.z
    );

    Eigen::Vector3d v_new = extrinsic.block<3,3>(0,0) * v_old;

    odom_out.twist.twist.linear.x = v_new.x();
    odom_out.twist.twist.linear.y = v_new.y();
    odom_out.twist.twist.linear.z = v_new.z();
    pub_global_world_odom.publish(odom_out);


    // 创建一个PoseStamped消息,用来输入 mavros 的 vision_pose 话题
    geometry_msgs::PoseStamped posestamp;
    posestamp.header.stamp = ros::Time::now();
    posestamp.header.frame_id = "uav_world";
    posestamp.pose.position.x =  trans_new.x();
    posestamp.pose.position.y =  trans_new.y();
    posestamp.pose.position.z =  trans_new.z();
    posestamp.pose.orientation.x =  q_new.x();
    posestamp.pose.orientation.y =  q_new.y();
    posestamp.pose.orientation.z =  q_new.z();
    posestamp.pose.orientation.w =  q_new.w();
    pub_odom_mavros.publish(posestamp);
    odom_enable = true;
}

void pathToUavWorldCallback(const nav_msgs::PathConstPtr& path_msg)
{
    nav_msgs::Path path_out = *path_msg;
    path_out.header.frame_id = "uav_world";

    for (size_t i = 0; i < path_out.poses.size(); ++i) {
        const geometry_msgs::PoseStamped& pose_stamped = path_msg->poses[i];

        Eigen::Quaterniond q(
            pose_stamped.pose.orientation.w,
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z
        );
        Eigen::Matrix3d rot = q.toRotationMatrix();
        Eigen::Vector3d trans(
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z
        );

        Eigen::Matrix4d pose_transform = Eigen::Matrix4d::Identity();
        pose_transform.block<3,3>(0,0) = rot;
        pose_transform.block<3,1>(0,3) = trans;

        Eigen::Matrix4d pose_in_uav_world = extrinsic * pose_transform * extrinsic.inverse();

        Eigen::Matrix3d rot_new = pose_in_uav_world.block<3,3>(0,0);
        Eigen::Vector3d trans_new = pose_in_uav_world.block<3,1>(0,3);
        Eigen::Quaterniond q_new(rot_new);

        path_out.poses[i].header.frame_id = "uav_world";
        path_out.poses[i].pose.position.x = trans_new.x();
        path_out.poses[i].pose.position.y = trans_new.y();
        path_out.poses[i].pose.position.z = trans_new.z();
        path_out.poses[i].pose.orientation.x = q_new.x();
        path_out.poses[i].pose.orientation.y = q_new.y();
        path_out.poses[i].pose.orientation.z = q_new.z();
        path_out.poses[i].pose.orientation.w = q_new.w();
    }

    // 发布转换后的轨迹
    pub_global_world_path.publish(path_out);
}

void cloudToUavWorldCallback(const sensor_msgs::PointCloud2::ConstPtr& input_msg)
{
    // 直接使用外参矩阵
    Eigen::Matrix4d transform = extrinsic;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *cloud_transformed, transform);

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_transformed, output_msg);
    output_msg.header = input_msg->header;
    output_msg.header.frame_id = "uav_world";
    pub_global_world_pc.publish(output_msg);
}

double diff_pose[3]={},dit_time=20;
double fast_pose_diff_X=0, fast_pose_diff_Y=0, fast_pose_diff_Z=0;
double get_roll=0, get_pitch=0, get_yaw=0;

// 用于估计无人机初始偏差位置
void odom_racercall(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    if( dit_time >0 ){//采样20次，耗时20/10=2秒
        diff_pose[0] +=  odom_msg->pose.pose.position.x;
        diff_pose[1] +=  odom_msg->pose.pose.position.y;
        diff_pose[2] +=  odom_msg->pose.pose.position.z;
        dit_time--;
    }
    else if( dit_time==0 ) 
    {
        diff_pose[0] /= 20;
        diff_pose[1] /= 20;
        diff_pose[2] /= 20;
        dit_time--;
        std::cout << "diff_pose: " << "x = " << diff_pose[0] << " y = "<< diff_pose[1] << " z = "<< diff_pose[2]<< " m" <<std::endl;
        ros::param::set("fast_odom_diff_X",diff_pose[0]);
        ros::param::set("fast_odom_diff_Y",diff_pose[1]);
        ros::param::set("fast_odom_diff_Z",diff_pose[2]);
        //四元素转欧拉角
	tf::Quaternion quat;
	tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(get_roll, get_pitch, get_yaw);
    get_roll*=57.3f;
    get_pitch*=57.3f;
    get_yaw*=57.3f;
    std::cout << "init_angle: rol = " << get_roll << " pit = "<< get_pitch<< " yaw = " << get_yaw << "  angle" <<std::endl;

        if( (get_roll >-5 && get_roll<5) && (get_pitch >-5 && get_pitch<5) && (get_yaw >-5 && get_yaw<5) )//假定初始化角度偏移 5度
        {
            if( (diff_pose[0] >-0.5 && diff_pose[0]<0.5) && (diff_pose[1] >-0.5 && diff_pose[1]<0.5) && (diff_pose[2] >-0.5 && diff_pose[2]<0.5) )
            ROS_INFO("drone_odom_succed");;
        }
        else
        {
            ROS_WARN("drone_odom_errors");
            system("rosnode kill -a");//关闭所有节点
        }
    }
    // 创建一个PoseStamped消息
    nav_msgs::Odometry odom_racer;
    if(dit_time < 0)
    {
        odom_racer.header.frame_id = "uav_world";
        odom_racer.header.stamp = odom_msg->header.stamp;
        odom_racer.child_frame_id = odom_racer_id;
        odom_racer.pose.pose.orientation =  odom_msg->pose.pose.orientation ;
        odom_racer.twist = odom_msg->twist;
        //除了这个基本都改
        odom_racer.pose.pose.position.x =  odom_msg->pose.pose.position.x + racer_x -diff_pose[0];
        odom_racer.pose.pose.position.y =  odom_msg->pose.pose.position.y + racer_y -diff_pose[1];
        odom_racer.pose.pose.position.z =  odom_msg->pose.pose.position.z + racer_z -diff_pose[2];
        // 发布PoseStamped消息
        pub_odom_racer.publish(odom_racer);

        geometry_msgs::PoseStamped posesracer;

        // 设置header
        posesracer.header.stamp = ros::Time::now();
        posesracer.header.frame_id = "uav_world";
        // 设置pose
        posesracer.pose.position.x =  odom_msg->pose.pose.position.x + racer_x -diff_pose[0];
        posesracer.pose.position.y =  odom_msg->pose.pose.position.y + racer_y -diff_pose[1];
        posesracer.pose.position.z =  odom_msg->pose.pose.position.z + racer_z -diff_pose[2];

        posesracer.pose.orientation.x =  odom_msg->pose.pose.orientation.x;
        posesracer.pose.orientation.y =  odom_msg->pose.pose.orientation.y;
        posesracer.pose.orientation.z =  odom_msg->pose.pose.orientation.z;
        posesracer.pose.orientation.w =  odom_msg->pose.pose.orientation.w;
        pub_pose_racer.publish(posesracer);

        ros::param::set("fast_pose_diff_X",posesracer.pose.position.x);
        ros::param::set("fast_pose_diff_Y",posesracer.pose.position.y);
        ros::param::set("fast_pose_diff_Z",posesracer.pose.position.z);
    }
}

ros::Time begin_time;
int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "data_manager");
    ros::NodeHandle nh;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    std::string base_name = ros::this_node::getName();
    
    // 订阅odom话题
    odom_sub_racer = nh.subscribe("/global_fastlio_odom", 1000, odom_racercall);
    sub_uav_odom = nh.subscribe<nav_msgs::Odometry>("/fastlio_odom", 10, odomToUavWorldCallback);
    sub_uav_path = nh.subscribe<nav_msgs::Path>("/fastlio_path", 10, pathToUavWorldCallback);
    sub_uav_pc = nh.subscribe<sensor_msgs::PointCloud2>("/fastlio_pointcloud", 10, cloudToUavWorldCallback);

    // 发布PoseStamped话题
    pub_odom_mavros = nh.advertise<geometry_msgs::PoseStamped>("odom_mavros", 1000);
    pub_odom_racer = nh.advertise<nav_msgs::Odometry>("odom_racer", 1000);
    pub_pose_racer = nh.advertise<geometry_msgs::PoseStamped>("pose_racer", 1000);
    pub_global_world_odom = nh.advertise<nav_msgs::Odometry>("/global_fastlio_odom", 1000);
    pub_global_world_path = nh.advertise<nav_msgs::Path>("/global_fastlio_path", 1000);
    pub_global_world_pc = nh.advertise<sensor_msgs::PointCloud2>("/global_fastlio_pointcloud", 1000);
    nh.getParam(base_name + "/racer_x",racer_x);
    nh.getParam(base_name + "/racer_y",racer_y);
    nh.getParam(base_name + "/racer_z",racer_z);
    nh.getParam(base_name + "/odom_racer_id",odom_racer_id);
    begin_time=ros::Time::now(); 
    ros::Rate rate(10);

    // 读取 extrinsic_R 和 extrinsic_T
    std::vector<double> extrinsic_R, extrinsic_T;
    if (nh.getParam("imu2lidar/extrinsic_R", extrinsic_R) && nh.getParam("imu2lidar/extrinsic_T", extrinsic_T)) {
        if (extrinsic_R.size() == 9 && extrinsic_T.size() == 3) {
            Eigen::Matrix3d R;
            R << extrinsic_R[0], extrinsic_R[1], extrinsic_R[2],
                 extrinsic_R[3], extrinsic_R[4], extrinsic_R[5],
                 extrinsic_R[6], extrinsic_R[7], extrinsic_R[8];
            Eigen::Vector3d T(extrinsic_T[0], extrinsic_T[1], extrinsic_T[2]);
            extrinsic.setIdentity();
            extrinsic.block<3,3>(0,0) = R;
            extrinsic.block<3,1>(0,3) = T;
            ROS_INFO_STREAM("Loaded extrinsic:\n" << extrinsic);
        } else {
            ROS_WARN("imu2lidar/extrinsic_R or extrinsic_T size error");
        }
    } else {
        ROS_WARN("Cannot load imu2lidar/extrinsic_R or extrinsic_T from parameter server, using identity.");
    }

    // 判定 mid360 能正常使用
    while (ros::ok())
    {
        std::string full_topic = nh.resolveName("livox/lidar"); // 自动补全命名空间，例如 /x280_1/livox/lidar
        if (topicExists(full_topic)) {
            ROS_INFO("Mid360 online!");
            break;
        }
        if ((ros::Time::now() - begin_time) > ros::Duration(1.0)) {
            ROS_INFO("WAIT mid360 online...");
        }
        rate.sleep();
    }

    // 判定fast-lio odom
    while(ros::ok())
    {
        if(odom_enable==true)
            break;
        if( (ros::Time::now()-begin_time) > ros::Duration(1.0)){
            ROS_INFO("WAIT fast-lio_odom");
        }
        // 为了重新获取 odom_mavroscall 回调
        ros::spinOnce(); 
        rate.sleep();
    }

    // 进入ROS事件循环
    ros::spin();
    return 0;
}
