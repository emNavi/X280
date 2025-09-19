#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
typedef pcl::PointXYZ PointType;

double local_x,local_y,local_z;
double update_x,update_y,update_z;
double downsampleCloud_max=0.1;
int Map_sampling_rate,Cloud_receive,Map_pub_flage;
bool clean_localmap=false;
class LocalMapUpdater
{
public:
    LocalMapUpdater(ros::NodeHandle& nh) : nh_(nh)
    {
        // 参数，可根据需求改
        region_global_ = Eigen::Vector3f(local_x, local_y, local_z);
        region_update_ = Eigen::Vector3f(update_x, update_y, update_z);

        sub_cloud_ = nh_.subscribe("/cloud_registered", 10, &LocalMapUpdater::cloudCallback, this);
        sub_odom_ = nh_.subscribe("/global_fastlio_odom", 50, &LocalMapUpdater::odomCallback, this);

        pub_local_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_map", 1);
        pub_updated_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/updated_map", 1);

        odom_received_ = false;
        local_map_.reset(new pcl::PointCloud<PointType>());
        updated_map_.reset(new pcl::PointCloud<PointType>());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        std::cout << "Received odometry message" << std::endl;
        latest_odom_ = *odom_msg;
        odom_received_ = true;
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        std::cout << "Received cloud message" << std::endl;

        if (!odom_received_) return;
        ros::param::get("clean_localmap",clean_localmap);
        Cloud_receive++;
        if( Cloud_receive%Map_sampling_rate == 0 )// 10
        {
            pcl::PointCloud<PointType>::Ptr cloud_in(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*cloud_msg, *cloud_in);

            // 初始化局部全局地图，裁剪到5*5*2范围
            updateLocalMap(cloud_in);       
            // 实时局部3*3*2区域更新
            updateDynamicRegion(cloud_in);
            //
            if(clean_localmap==true)
            {
                local_map_.reset(new pcl::PointCloud<PointType>());
                updated_map_.reset(new pcl::PointCloud<PointType>());
                std::cout <<"clean map"<< std::endl;
            }
            else
                publishMaps();//10hz
            // std::cout << "local_map"<<std::endl;
        }
        if(Cloud_receive%10 == 0 )// clean_localmap to time 1s   
            ros::param::set("clean_localmap",false);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_cloud_, sub_odom_;
    ros::Publisher pub_local_map_, pub_updated_map_;

    nav_msgs::Odometry latest_odom_;
    bool odom_received_;

    pcl::PointCloud<PointType>::Ptr local_map_;   // 5x5x2 m范围静态累积地图
    pcl::PointCloud<PointType>::Ptr updated_map_; // 3x3x2 m局部更新地图

    Eigen::Vector3f region_global_;
    Eigen::Vector3f region_update_;

    Eigen::Matrix4f odomMsgToEigen(const nav_msgs::Odometry& odom)
    {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        const auto& p = odom.pose.pose.position;
        const auto& q = odom.pose.pose.orientation;
        Eigen::Quaternionf qf(q.w, q.x, q.y, q.z);
        T.block<3,3>(0,0) = qf.toRotationMatrix();
        T(0,3) = p.x;
        T(1,3) = p.y;
        T(2,3) = p.z;
        return T;
    }

    void downsampleCloud(pcl::PointCloud<PointType>::Ptr& cloud, float leaf_size = 0.1)
    {
        pcl::VoxelGrid<PointType> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
        pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
        voxel.filter(*filtered);
        cloud = filtered;
    }

    void updateLocalMap(const pcl::PointCloud<PointType>::Ptr& cloud_w)
    {
        // 限定local_map_在5x5x2m范围内，裁剪中心为当前里程计位置
        Eigen::Vector3f center(latest_odom_.pose.pose.position.x,
                               latest_odom_.pose.pose.position.y,
                               latest_odom_.pose.pose.position.z);
        // 裁剪当前帧点云
        pcl::CropBox<PointType> crop_filter;
        crop_filter.setInputCloud(cloud_w);
        crop_filter.setMin(Eigen::Vector4f(center.x() - region_global_.x()/2,
                                           center.y() - region_global_.y()/2,
                                           center.z() - region_global_.z()/2,
                                           1.0));
        crop_filter.setMax(Eigen::Vector4f(center.x() + region_global_.x()/2,
                                           center.y() + region_global_.y()/2,
                                           center.z() + region_global_.z()/2,
                                           1.0));

        pcl::PointCloud<PointType>::Ptr cropped_frame(new pcl::PointCloud<PointType>());
        crop_filter.filter(*cropped_frame);

        // Step 2: 合入地图
        *local_map_ += *cropped_frame;
        
        // Step 3: 再裁剪整个地图，避免过多累积
        crop_filter.setInputCloud(local_map_);
        pcl::PointCloud<PointType>::Ptr cropped_map(new pcl::PointCloud<PointType>());
        crop_filter.filter(*cropped_map);
        local_map_ = cropped_map;

        // Step 4: 降采样地图
        downsampleCloud(local_map_, 0.1); // 可调精度
    }

    void updateDynamicRegion(const pcl::PointCloud<PointType>::Ptr& cloud_w)
    {
        // 先从local_map_中删除以当前位置为中心的3*3*2m区域
        Eigen::Vector3f center(latest_odom_.pose.pose.position.x,
                               latest_odom_.pose.pose.position.y,
                               latest_odom_.pose.pose.position.z);

        pcl::CropBox<PointType> crop_filter;
        crop_filter.setInputCloud(local_map_);
        crop_filter.setMin(Eigen::Vector4f(center.x() - region_update_.x()/2,
                                           center.y() - region_update_.y()/2,
                                           center.z() - region_update_.z()/2,
                                           1.0));
        crop_filter.setMax(Eigen::Vector4f(center.x() + region_update_.x()/2,
                                           center.y() + region_update_.y()/2,
                                           center.z() + region_update_.z()/2,
                                           1.0));
        crop_filter.setNegative(true); // 删除此区域

        pcl::PointCloud<PointType>::Ptr map_without_region(new pcl::PointCloud<PointType>());
        crop_filter.filter(*map_without_region);

        // 合并新点云（不重复局部区域）
        *map_without_region += *cloud_w;
        downsampleCloud(map_without_region, 0.1); // 也做降采样
        updated_map_ = map_without_region;
    }

    void publishMaps()
    {
        sensor_msgs::PointCloud2 msg_local, msg_updated;
        pcl::toROSMsg(*local_map_, msg_local);
        pcl::toROSMsg(*updated_map_, msg_updated);

        msg_local.header.frame_id = "uav_world";
        msg_local.header.stamp = ros::Time::now();

        msg_updated.header.frame_id = "uav_world";
        msg_updated.header.stamp = ros::Time::now();

        pub_local_map_.publish(msg_local);
        pub_updated_map_.publish(msg_updated);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_map_node");
    ros::NodeHandle nh;

    nh.param("local_x",local_x,5.0);
    nh.param("local_y",local_y,5.0);
    nh.param("local_z",local_z,5.0);
    nh.param("update_x",update_x,3.0);
    nh.param("update_y",update_y,3.0);
    nh.param("update_z",update_z,3.0);
    nh.param("Map_sampling_rate",Map_sampling_rate,1);

    nh.param("downsampleCloud_max",downsampleCloud_max,0.1);

    nh.param("clean_localmap",clean_localmap,false);
    LocalMapUpdater updater(nh);
    std::cout << "local_map"<<std::endl;
    ros::spin();
    return 0;
}
