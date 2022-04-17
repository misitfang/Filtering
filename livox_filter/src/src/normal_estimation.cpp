#include "normal_estimation.h"

Normal_estimation::Normal_estimation()
{
    std::cout << "................................" << std::endl;
    initros();
}

void Normal_estimation::initros()
{
    std::cout << "Ros Initialize no debug" << std::endl;
    ROS_DEBUG_STREAM("Ros Initialize");
    std::string cloud_input_, cloud_output_,cloud_output_cetract;

    node_handle_.param<std::string>("/livox_filter/cloud_input", cloud_input_, "/livox/lidar");
    ROS_INFO_STREAM("cloud_in:" << cloud_input_);
    node_handle_.param<double>("/livox_filter/threshold_height", threshold_height_, 1);
    node_handle_.param<std::string>("/livox_filter/cloud_no_ground", cloud_output_, "/cloud_no_ground");
    node_handle_.param<std::string>("/livox_filter/cloud_no_ground", cloud_output_cetract, "/cloud_output_cetract");

    sub_cloud = node_handle_.subscribe(cloud_input_, 100, &Normal_estimation::cloud_callback, this);
    pub_no_ground = node_handle_.advertise<sensor_msgs::PointCloud2>(cloud_output_, 100);
    pub_extract = node_handle_.advertise<sensor_msgs::PointCloud2>(cloud_output_cetract, 100);

}

void Normal_estimation::calculate_theta_diff(pointcloudT::Ptr &cloud_in, pointnormalT::Ptr &normal_in, pointcloudT::Ptr &cloud_out)
{
    if (cloud_in->empty() || cloud_out->empty())
        return;
    else
    {
        for (size_t i = 0; i < cloud_in->size(); i++)
        {
            pointT p = cloud_in->points[i];
            p.intensity = acos(normal_in->points[i].normal_x * normal_model[0] +
                               normal_in->points[i].normal_y * normal_model[1] + normal_in->points[i].normal_z * normal_model[2]);
            p.intensity *= prad;
            cloud_out->points.push_back(p);
        }
        return;
    }
}
void Normal_estimation::filter_cloud(pointcloudT::Ptr &cloud_in, pointcloudT::Ptr &cloud_ground, pointcloudT::Ptr &cloud_no_ground)
{
    if (cloud_in->empty())
        return;
    else
    {
        pcl::ExtractIndices<pointT> ex;
        // pcl::PointIndices indices;
        std::vector<int> indices;
        ex.setInputCloud(cloud_in);

        for (size_t i = 0; i < cloud_in->size(); i++)
        {
            if (0 < cloud_in->points[i].intensity && cloud_in->points[i].intensity < 20)
            {
                indices.push_back(i);
            }
        }
        ROS_INFO_STREAM("indices size: " << indices.size());
        ex.setIndices(boost::make_shared<std::vector<int>>(indices));
        ex.setNegative(false);
        ex.filter(*cloud_ground);
        ex.setNegative(true);
        ex.filter(*cloud_no_ground);
        return;
    }
}
void Normal_estimation::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_in)
{
    // std::cout<<"cloud_callback"<<std::endl;
    pointcloudT::Ptr cloud_in(new pointcloudT());
    pcl::fromROSMsg(*cloud_msg_in, *cloud_in);

    pcl::ExtractIndices<pointT> extract;
    pointcloudT::Ptr cloud_extract(new pointcloudT);
    extract.setInputCloud(cloud_in);
    pcl::PointIndices indices;
    for (size_t i = 0; i < cloud_in->points.size(); i++)
    {
        if (cloud_in->points[i].z < threshold_height_)
        {
            indices.indices.push_back(i);
        }
    }
    extract.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    extract.setNegative(false);
    extract.filter(*cloud_extract);

    sensor_msgs::PointCloud2::Ptr extract_msg_cloud(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_extract, *extract_msg_cloud);
    extract_msg_cloud->header.stamp = cloud_msg_in->header.stamp;
    extract_msg_cloud->header.frame_id = cloud_msg_in->header.frame_id;
    pub_no_ground.publish(*extract_msg_cloud);

    pointnormalT::Ptr normal_cloud(new pointnormalT);
    pcl::NormalEstimation<pointT, normalT> ne;
    pcl::search::KdTree<pointT>::Ptr tree(new pcl::search::KdTree<pointT>);
    ne.setInputCloud(cloud_in);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.5);
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    ne.compute(*normal_cloud);

    pointcloudT::Ptr cloud_theta_diff(new pointcloudT());
    calculate_theta_diff(cloud_in, normal_cloud, cloud_theta_diff);

    pointcloudT::Ptr cloud_no_ground(new pointcloudT());
    pointcloudT::Ptr cloud_ground(new pointcloudT());

    filter_cloud(cloud_theta_diff, cloud_ground, cloud_no_ground);
    sensor_msgs::PointCloud2::Ptr no_ground_msgs(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_no_ground, *no_ground_msgs);
    no_ground_msgs->header.stamp = cloud_msg_in->header.stamp;
    no_ground_msgs->header.frame_id = cloud_msg_in->header.frame_id;
    pub_no_ground.publish(*no_ground_msgs);
}