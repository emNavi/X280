#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <string>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>

ros::Subscriber state_sub_,control_state_sub_;
ros::Publisher pub_mavros_state, pub_control_state;

// ros::Subscriber swarm_takeoff_sub_,swarm_land_sub_;
// ros::Publisher pub_swarm_takeoff,pub_swarm_land;

int swarmtakeoffCallback_flage = 0, swarmlandCallback_flage = 0;
void stateCallback(const mavros_msgs::State::ConstPtr& msg){
   pub_mavros_state.publish(msg);
}
void controlStateCallback(const std_msgs::String::ConstPtr& msg){
    pub_control_state.publish(msg);
}

// void swarmtakeoffCallback(const std_msgs::String::ConstPtr& msg){
//     swarmtakeoffCallback_flage++;
//     if( (swarmtakeoffCallback_flage+5)%5 != 0)     pub_swarm_takeoff.publish(msg);
// }
// void swarmlandCallback(const std_msgs::String::ConstPtr& msg){
//     swarmlandCallback_flage++;
//     if( (swarmlandCallback_flage+5)%5 != 0)     pub_swarm_land.publish(msg);
// }

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "racer_topic");
    ros::NodeHandle nh_;
    
    state_sub_ = nh_.subscribe("/mavros/state", 10, stateCallback);
    control_state_sub_  = nh_.subscribe("/ctrl_bridge/bridge_status", 10, controlStateCallback);
    pub_mavros_state    = nh_.advertise<mavros_msgs::State>("/drone_1/mavros/state",10);
    pub_control_state   = nh_.advertise<std_msgs::String>("/drone_1/ctrl_bridge/bridge_status",10);

    // swarm_takeoff_sub_  = nh_.subscribe("/conn2x/emnavi_cmd/takeoff", 10, swarmtakeoffCallback);
    // swarm_land_sub_  = nh_.subscribe("/conn2x/emnavi_cmd/land", 10, swarmlandCallback);
    // pub_swarm_takeoff   = nh_.advertise<std_msgs::String>("/emnavi_cmd/takeoff",10);
    // pub_swarm_land   = nh_.advertise<std_msgs::String>("/emnavi_cmd/land",10);
    // 进入ROS事件循环
    ros::spin();

    return 0;
}