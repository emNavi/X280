#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class ImageCompressor {
public:
    ImageCompressor(ros::NodeHandle& nh) {
        sub_ = nh.subscribe("camera/color/image_raw", 1, &ImageCompressor::imageCallback, this);
        pub_ = nh.advertise<sensor_msgs::CompressedImage>("camera/color/image_ultra_compressed", 1);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImageConstPtr cv_ptr;
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");

            std::vector<uchar> buffer;
            std::vector<int> params;

            // 使用 WebP 压缩
            params.push_back(cv::IMWRITE_WEBP_QUALITY);
            params.push_back(30); // 质量 0-100，越低越小

            bool ok = cv::imencode(".webp", cv_ptr->image, buffer, params);

            if (!ok) {
                ROS_WARN("Failed to encode image!");
                return;
            }

            sensor_msgs::CompressedImage out_msg;
            out_msg.header = msg->header;
            out_msg.format = "webp";
            out_msg.data = buffer;

            pub_.publish(out_msg);

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

// TODO(): 暂未使用
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // 转换为 PCL 格式（不带颜色）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // TODO: 在这里添加点云处理代码
    static ros::Publisher pub;
    static bool initialized = false;
    if (!initialized) {
        ros::NodeHandle nh;
        pub = nh.advertise<sensor_msgs::PointCloud2>("/uav_compress_pointcloud", 10);
        initialized = true;
    }

    // 过滤点云：只保留高度在 0.0 到 2.0 米之间的点
    // 设定高度区间
    double min_z = -0.8; // 最小高度
    double max_z = 1.5; // 最大高度

    // ----------------- Z 轴过滤 -----------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : cloud->points) {
        if (pt.z >= min_z && pt.z <= max_z) {
            temp_cloud->points.push_back(pt);
        }
    }

    // // ----------------- 空洞/降采样处理 -----------------
    // // 使用 VoxelGrid 统一点云密度，顺便补全空洞
    // pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::VoxelGrid<pcl::PointXYZ> voxel;
    // voxel.setInputCloud(temp_cloud);
    // voxel.setLeafSize(0.01f, 0.01f, 0.01f); // 1cm体素
    // voxel.filter(*voxel_cloud);

    // ----------------- 去除孤立点 -----------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr denoised_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(temp_cloud);
    sor.setMeanK(20);               // 每个点考虑 20 个邻居
    sor.setStddevMulThresh(1.0);    // 超出均值 1.0倍标准差的点删除
    sor.filter(*denoised_cloud);

    // 直接发布处理后的点云（去噪后）
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*denoised_cloud, output_msg);
    output_msg.header = cloud_msg->header;
    pub.publish(output_msg);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "msg_compress_send");
    ros::NodeHandle nh;
    ImageCompressor compressor(nh);
    // ros::Subscriber sub = nh.subscribe("global_fastlio_pointcloud_2", 10, cloudCallback);
    // ros::Subscriber img_sub = nh.subscribe("camera/color/image_raw/compressed", 10, imageCallback);
    // ROS_INFO("Listening to /cloud_registered ...");
    ros::spin();
    return 0;
}