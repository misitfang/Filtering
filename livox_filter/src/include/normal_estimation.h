#ifndef _NORMAL_ESTIMATION_
#define _NORMAL_ESTIMATION_

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#define pi 3.1415926
#define prad 180/pi

typedef pcl::PointXYZI pointT;
typedef pcl::PointCloud<pointT> pointcloudT;
typedef pcl::PointNormal normalT;
typedef pcl::PointCloud<normalT> pointnormalT;

class Normal_estimation
{
private:
    double threshold_height_;
    std::vector<double> normal_model;
    ros::NodeHandle node_handle_;
    ros::Publisher pub_no_ground;
    ros::Subscriber sub_cloud;
    ros::Publisher pub_extract;

    void initros();
    void calculate_theta_diff(pointcloudT::Ptr &cloud_in, pointnormalT::Ptr &normal_in, pointcloudT::Ptr &cloud_out);
    void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_in);
    void filter_cloud(pointcloudT::Ptr &cloud_in,pointcloudT::Ptr &cloud_ground,pointcloudT::Ptr &cloud_no_ground);

public:
    Normal_estimation();
};
#endif