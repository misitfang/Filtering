#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

class Space_filter
{
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;

    std::string subscribe_topic_;
    bool later_removal_;
    bool vertical_removal_;
    double left_distance_;
    double right_distance_;
    double below_distance_;
    double above_distance_;
    void clip_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float above_distance_threshold, float below_distance_threshold);
    void point_callback(const sensor_msgs::PointCloud2::Ptr &in_sensor_cloud_ptr);
    void KeepLanes(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_left_lane_threshold, float in_right_lane_threshold);

public:
    Space_filter();
};
void Space_filter::clip_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
 float above_distance_threshold, float below_distance_threshold)
{
    out_cloud_ptr->points.clear();
    for (unsigned i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        if (in_cloud_ptr->points[i].z <= above_distance_threshold && 
        in_cloud_ptr->points[i].z >= below_distance_threshold)
        {
            out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
        }
    }
}
void Space_filter::KeepLanes(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_left_lane_threshold, float in_right_lane_threshold)
{
    pcl::PointIndices::Ptr far_indices(new pcl::PointIndices);
    for (unsigned i; i < in_cloud_ptr->size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_cloud_ptr->points[i].x;
        current_point.y = in_cloud_ptr->points[i].y;
        current_point.z = in_cloud_ptr->points[i].z;
        if (current_point.y > in_left_lane_threshold || current_point.y < -1 * in_right_lane_threshold)
        {
            far_indices->indices.push_back(i);
        }
        out_cloud_ptr->points.clear();
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(in_cloud_ptr);
        extract.setIndices(far_indices);
        extract.setNegative(true);
        extract.filter(*out_cloud_ptr);
    }
}

void Space_filter::point_callback(const sensor_msgs::PointCloud2::Ptr &in_sensor_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlanes_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*in_sensor_cloud_ptr, *current_sensor_cloud_ptr);

    if (later_removal_)
    {
        KeepLanes(current_sensor_cloud_ptr, inlanes_cloud_ptr, left_distance_, right_distance_);
    }
    else
    {
        inlanes_cloud_ptr = current_sensor_cloud_ptr;
    }
    if (vertical_removal_)
    {
        clip_cloud(inlanes_cloud_ptr, clipped_cloud_ptr, above_distance_, below_distance_);
    }
    else
    {
        clipped_cloud_ptr = inlanes_cloud_ptr;
    }
    sensor_msgs::PointCloud2 cloud_msgs;
    pcl::toROSMsg(*clipped_cloud_ptr,cloud_msgs);
    cloud_msgs.header=in_sensor_cloud_ptr->header;
    cloud_pub_.publish(cloud_msgs);


}

Space_filter::Space_filter() : node_handle_("~")
{
    node_handle_.param<std::string>("subscribe_topic", subscribe_topic_, "/hesai/pandar");
    node_handle_.param("later_removal", later_removal_, true);
    node_handle_.param("vertical_removal", vertical_removal_, true);
    node_handle_.param<double>("left_distance", left_distance_, 5);
    node_handle_.param<double>("right_distance", right_distance_, 5);
    node_handle_.param<double>("below_distance", below_distance_, 1);
    node_handle_.param<double>("above_distance", above_distance_, 1);

    cloud_sub_ = node_handle_.subscribe(subscribe_topic_, 10, &Space_filter::point_callback, this);
    cloud_pub_ =node_handle_.advertise<sensor_msgs::PointCloud2>( "/points_clipped", 10);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "space_filter");
    Space_filter node;
    ros::spin();
}
