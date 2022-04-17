#ifndef RAY_GROUND_FILTER_H
#define RAY_GROUND_FILTER_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl-1.8/pcl/features/normal_3d.h>
#include <pcl-1.8/pcl/features/normal_3d_omp.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>

#include <pcl_ros/transforms.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <opencv2/core/version.hpp>

#include <dynamic_reconfigure/server.h>
#include "ray_ground_filter/RayConfig.h"

class Ray_ground_filter
{
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber point_node_sub_;
    ros::Publisher groundless_points_pub_;
    ros::Publisher ground_points_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string input_point_topic_;
    std::string base_frame_;

    double general_max_slope_; //是我们设定的同条射线上邻近两点的坡度阈值
    double local_max_slope_;   //表示整个地面的坡度阈值，这两个坡度阈值的单位为度
    //通过这两个坡度阈值以及当前点的半径（到lidar的水平距离）求得高度阈值，
    //通过判断当前点的高度（即点的z值）是否在地面加减高度阈值范围内来判断当前点是为地面。
    double clipping_height_;
    double min_point_distance_;
    double radial_divider_angle_;
    double concentric_divider_distance_;
    double min_height_threshold_;
    double reclass_distance_threshold_;

    size_t radial_dividers_num_;
    size_t concentric_dividers_num_;

    // std::vector<cv::Scalar> colors_;
    const size_t color_num_ = 60; // different number of color to generate

    struct PointXYZIRTColor
    {
        pcl::PointXYZI point;

        float radius; // cylindric coords on XY Plane
        float theta;  // angle deg on XY plane

        size_t radial_div;     // index of the radial divsion to which this point belongs to角度微分
        size_t concentric_div; // index of the concentric division to which this points belongs to距离微分

        size_t red;   // Red component  [0-255]
        size_t green; // Green Component[0-255]
        size_t blue;  // Blue component [0-255]

        size_t original_index; // index of this point in the source pointcloud
    };
    typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

    bool TransformPointCloud(const std::string &in_target_frame, const sensor_msgs::PointCloud2::ConstPtr &in_cloud_ptr,
                             const sensor_msgs::PointCloud2::Ptr &out_cloud_ptr);
    void publish_cloud(const ros::Publisher &in_publisher,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                       const std_msgs::Header &in_header);

    void ExtractPointsIndices(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                              const pcl::PointIndices &in_indices,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr out_only_indices_cloud_ptr,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr out_removed_indices_cloud_ptr);

    void ClassifyPointCloud(const std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                            const pcl::PointIndices::Ptr &out_ground_indices,
                            const pcl::PointIndices::Ptr &out_no_ground_indices);

    void ConvertXYZIToRTZColor(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
        const std::shared_ptr<PointCloudXYZIRTColor> &out_organized_points,
        const std::shared_ptr<std::vector<pcl::PointIndices>> &out_radial_divided_indices,
        const std::shared_ptr<std::vector<PointCloudXYZIRTColor>> &out_radial_ordered_clouds);

    void RemovePointsUpTo(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, double in_min_distance,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr out_filtered_cloud_ptr);

    void ClipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, const double in_clip_height,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr out_clipped_cloud_ptr);

    void CloudCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);

    dynamic_reconfigure::Server<ray_ground_filter::RayConfig> *dsrv_;

    void reconfigureCB(ray_ground_filter::RayConfig&config, uint32_t level);

public:
    Ray_ground_filter();
    void Run();
};

#endif
