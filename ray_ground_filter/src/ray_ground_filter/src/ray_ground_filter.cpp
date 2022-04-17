#include "ray_ground_filter.h"

using namespace std;

bool Ray_ground_filter::TransformPointCloud(const std::string &in_target_frame,
                                            const sensor_msgs::PointCloud2::ConstPtr &in_cloud_ptr,
                                            const sensor_msgs::PointCloud2::Ptr &out_cloud_ptr)
{
    if (in_target_frame == in_cloud_ptr->header.frame_id)
    {
        *out_cloud_ptr = *in_cloud_ptr;
        return true;
    }

    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer_.lookupTransform(in_target_frame, in_cloud_ptr->header.frame_id,
                                                       in_cloud_ptr->header.stamp, ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return false;
    }
    // tf2::doTransform(*in_cloud_ptr, *out_cloud_ptr, transform_stamped);
    Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(mat, *in_cloud_ptr, *out_cloud_ptr);
    out_cloud_ptr->header.frame_id = in_target_frame;
    return true;
}

void Ray_ground_filter::publish_cloud(const ros::Publisher &in_publisher,
                                      const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                                      const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2::Ptr cloud_msg_ptr(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2::Ptr trans_cloud_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*in_cloud_to_publish_ptr, *cloud_msg_ptr);
    cloud_msg_ptr->header.frame_id = base_frame_;
    cloud_msg_ptr->header.stamp = in_header.stamp;
    const bool succeeded = TransformPointCloud(in_header.frame_id, cloud_msg_ptr, trans_cloud_msg_ptr);
    if (!succeeded)
    {
        ROS_ERROR_STREAM_THROTTLE(100, "Failed transform from " << cloud_msg_ptr->header.frame_id << " to "
                                                                << in_header.frame_id);
        return;
    }
    in_publisher.publish(*trans_cloud_msg_ptr);
}

void Ray_ground_filter::ExtractPointsIndices(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                             const pcl::PointIndices &in_indices,
                                             pcl::PointCloud<pcl::PointXYZI>::Ptr out_only_indices_cloud_ptr,
                                             pcl::PointCloud<pcl::PointXYZI>::Ptr out_removed_indices_cloud_ptr)
{
    pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
    extract_ground.setInputCloud(in_cloud_ptr);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(in_indices));

    extract_ground.setNegative(false); // true removes the indices, false leaves only the indices
    extract_ground.filter(*out_only_indices_cloud_ptr);

    extract_ground.setNegative(true); // true removes the indices, false leaves only the indices
    extract_ground.filter(*out_removed_indices_cloud_ptr);
}

void Ray_ground_filter::ClassifyPointCloud(const std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                                           const pcl::PointIndices::Ptr &out_ground_indices,
                                           const pcl::PointIndices::Ptr &out_no_ground_indices)
{
    out_ground_indices->indices.clear();    //地面店
    out_no_ground_indices->indices.clear(); //非地面点
#pragma omp for
    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) // 遍历每一根射线
    {
        float prev_radius = 0.f;
        float prev_height = 0.f;
        bool prev_ground = false;
        bool current_ground = false;
        //遍历每一根射线上的所有点
        for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) // loop through each point in the radial div
        {
            float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius; //与前一个点之间的距离
            float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
            float current_height = in_radial_ordered_clouds[i][j].point.z; //当前点实际z值
            float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;

            // for points which are very close causing the height threshold to be tiny, set a minimum value
            // 对于非常接近导致高度阈值很小的点，设置一个最小值
            if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
            {
                height_threshold = min_height_threshold_;
            }

            // check current point height against the LOCAL threshold (previous point)对照局部阈值检查当前点高度
            if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
            {
                // Check again using general geometry (radius from origin) if previous points wasn't ground
                if (!prev_ground)
                {
                    if (current_height <= general_height_threshold && current_height >= -general_height_threshold)
                    {
                        current_ground = true;
                    }
                    else
                    {
                        current_ground = false;
                    }
                }
                else
                {
                    current_ground = true;
                }
            }
            else
            {
                // check if previous point is too far from previous one, if so classify again
                if (points_distance > reclass_distance_threshold_ &&
                    (current_height <= height_threshold && current_height >= -height_threshold))
                {
                    current_ground = true;
                }
                else
                {
                    current_ground = false;
                }
            }

            if (current_ground)
            {
                out_ground_indices->indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            }
            else
            {
                out_no_ground_indices->indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = false;
            }

            prev_radius = in_radial_ordered_clouds[i][j].radius;
            prev_height = in_radial_ordered_clouds[i][j].point.z;
        }
    }
}

void Ray_ground_filter::ConvertXYZIToRTZColor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
    const std::shared_ptr<PointCloudXYZIRTColor> &out_organized_points,
    const std::shared_ptr<std::vector<pcl::PointIndices>> &out_radial_divided_indices,
    const std::shared_ptr<std::vector<PointCloudXYZIRTColor>> &out_radial_ordered_clouds)
{
    out_organized_points->resize(in_cloud->points.size());
    out_radial_divided_indices->clear();
    out_radial_divided_indices->resize(radial_dividers_num_);
    out_radial_ordered_clouds->resize(radial_dividers_num_);

    for (size_t i = 0; i < in_cloud->points.size(); i++)
    {
        PointXYZIRTColor new_point;
        auto radius = static_cast<float>(
            sqrt(in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y));
        auto theta = static_cast<float>(atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI);
        if (theta < 0)
        {
            theta += 360;
        }
        if (theta >= 360)
        {
            theta -= 360;
        }

        auto radial_div = (size_t)floor(theta / radial_divider_angle_);

        auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

        new_point.point = in_cloud->points[i];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radial_div = radial_div;
        new_point.concentric_div = concentric_div;
        // new_point.red = (size_t)colors_[new_point.radial_div % color_num_].val[0];
        // new_point.green = (size_t)colors_[new_point.radial_div % color_num_].val[1];
        // new_point.blue = (size_t)colors_[new_point.radial_div % color_num_].val[2];
        new_point.original_index = i;

        out_organized_points->at(i) = new_point;

        // radial divisions
        out_radial_divided_indices->at(radial_div).indices.push_back(i);

        out_radial_ordered_clouds->at(radial_div).push_back(new_point);
    } // end for

// order radial points on each division
#pragma omp for
    for (size_t i = 0; i < radial_dividers_num_; i++)
    {
        std::sort(out_radial_ordered_clouds->at(i).begin(), out_radial_ordered_clouds->at(i).end(),
                  [](const PointXYZIRTColor &a, const PointXYZIRTColor &b)
                  { return a.radius < b.radius; }); // NOLINT
    }
}

void Ray_ground_filter::RemovePointsUpTo(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, double in_min_distance,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr out_filtered_cloud_ptr)
{
    pcl::ExtractIndices<pcl::PointXYZI> extractor;
    extractor.setInputCloud(in_cloud_ptr);
    pcl::PointIndices indices;

    // #pragma omp for
    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        if (sqrt(in_cloud_ptr->points[i].x * in_cloud_ptr->points[i].x +
                 in_cloud_ptr->points[i].y * in_cloud_ptr->points[i].y) < in_min_distance)
        {
            indices.indices.push_back(i);
        }
    }
    extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    extractor.setNegative(true); // true removes the indices, false leaves only the indices
    extractor.filter(*out_filtered_cloud_ptr);
}

void Ray_ground_filter::ClipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, const double in_clip_height,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr out_clipped_cloud_ptr)
{
    pcl::ExtractIndices<pcl::PointXYZI> extractor; //创建滤波器对象
    extractor.setInputCloud(in_cloud_ptr);
    pcl::PointIndices indices;
//表示接下来的for循环将被多线程执行，另外每次循环之间不能有关系
#pragma omp for
    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        if (in_cloud_ptr->points[i].z > in_clip_height)
        {
            indices.indices.push_back(i);
        }
    }
    extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    extractor.setNegative(true); // true removes the indices, false leaves only the indices
    extractor.filter(*out_clipped_cloud_ptr);
}

void Ray_ground_filter::CloudCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud)
{
    //输入初始传感器点云坐标及点云，输出sensor::pointcloud类型点云
    sensor_msgs::PointCloud2::Ptr trans_sensor_cloud(new sensor_msgs::PointCloud2);
    const bool succeeded = TransformPointCloud(base_frame_, in_sensor_cloud, trans_sensor_cloud);
    if (!succeeded)
    {
        ROS_ERROR_STREAM_THROTTLE(10, "Failed transform from " << base_frame_ << " to "
                                                               << in_sensor_cloud->header.frame_id);
        return;
    }

    //Convert a PointCloud2 binary data blob into a pcl::PointCloud<T> object using a field_map.
    //sensor::pointcloud(trans_sensor_cloud) 2 pcl::pointcloud<pcl::pointxyzi>(current_sensor_cloud_ptr)
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*trans_sensor_cloud, *current_sensor_cloud_ptr);

    //删除设定高度之上部分的点云
    //current_sensor_cloud_ptr clipping_height_  out:clipped_cloud_ptr
    pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    ClipCloud(current_sensor_cloud_ptr, clipping_height_, clipped_cloud_ptr);

    //删除雷达附近设定阈值范围内的点云
    //in:clipped_cloud_ptr, min_point_distance_   out: filtered_cloud_ptr
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    RemovePointsUpTo(clipped_cloud_ptr, min_point_distance_, filtered_cloud_ptr);

    //输入filtered_cloud_ptr点云 转换xyzi2RTZcolor
    std::shared_ptr<PointCloudXYZIRTColor> organized_points(new PointCloudXYZIRTColor);
    std::shared_ptr<std::vector<pcl::PointIndices>> radial_division_indices(new std::vector<pcl::PointIndices>);
    std::shared_ptr<std::vector<PointCloudXYZIRTColor>> radial_ordered_clouds(new std::vector<PointCloudXYZIRTColor>);
    radial_dividers_num_ = ceil(360 / radial_divider_angle_);
    ConvertXYZIToRTZColor(filtered_cloud_ptr, organized_points, radial_division_indices, radial_ordered_clouds);

    //计算判断地面点与非地面点
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices), no_ground_indices(new pcl::PointIndices);
    ClassifyPointCloud(*radial_ordered_clouds, ground_indices, no_ground_indices);
    
    //滤波，
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    ExtractPointsIndices(filtered_cloud_ptr, *ground_indices, ground_cloud_ptr, no_ground_cloud_ptr);

    publish_cloud(ground_points_pub_, ground_cloud_ptr, in_sensor_cloud->header);
    publish_cloud(groundless_points_pub_, no_ground_cloud_ptr, in_sensor_cloud->header);
}
Ray_ground_filter::Ray_ground_filter() : node_handle_("~"), tf_listener_(tf_buffer_)
{
}

void Ray_ground_filter::reconfigureCB(ray_ground_filter::RayConfig &config, uint32_t level)
{
    input_point_topic_ = config.input_point_topic.c_str();
    base_frame_ = config.base_frame.c_str();
    clipping_height_ = config.clipping_height;
    local_max_slope_ = config.local_max_slope;
    radial_divider_angle_ = config.radial_divider_angle;
    concentric_divider_distance_ = config.concentric_divider_distance;
    min_height_threshold_ = config.min_height_threshold;
    min_point_distance_ = config.min_point_distance;
    reclass_distance_threshold_ = config.reclass_distance_threshold;
    general_max_slope_ = config.general_max_slope;

    ROS_INFO("clipping_height_ %f", config.clipping_height);
    ROS_INFO("local_max_slope %f", config.local_max_slope);
}

void Ray_ground_filter::Run()
{

    node_handle_.param<string>("input_point_topic", input_point_topic_, "/points_raw");
    // ROS_INFO("Input point_topic: %s", input_point_topic_.c_str());

    node_handle_.param<string>("base_frame", base_frame_, "base_link");
    // ROS_INFO("base_frame: %s", base_frame_.c_str());

    // node_handle_.param("general_max_slope", general_max_slope_, 3.0);
    // ROS_INFO("general_max_slope[deg]: %f", general_max_slope_);

    // node_handle_.param("local_max_slope", local_max_slope_, 5.0);
    // ROS_INFO("local_max_slope[deg]: %f", local_max_slope_);

    // node_handle_.param("radial_divider_angle", radial_divider_angle_, 0.1);
    // ROS_INFO("radial_divider_angle[deg]: %f", radial_divider_angle_);

    // node_handle_.param("concentric_divider_distance", concentric_divider_distance_, 0.0);
    // ROS_INFO("concentric_divider_distance[deg]: %f", concentric_divider_distance_);

    // node_handle_.param("min_heightun([deg]: %f", min_height_threshold_);

    // node_handle_.param("/ray_ground_filter/clipping_height", clipping_height_, 2.0);
    // ROS_INFO("clipping_height[deg]: %f", clipping_height_);

    // node_handle_.param("min_point_distance", min_point_distance_, 1.85);
    // ROS_INFO("min_point_distance[deg]: %f", min_point_distance_);

    // node_handle_.param("reclass_distance_threshold", reclass_distance_threshold_, 0.2); // 0.5 meters default
    // ROS_INFO("reclass_distance_threshold[deg]: %f", reclass_distance_threshold_);

    std::string no_ground_topic, ground_topic;
    node_handle_.param<std::string>("no_ground_point_topic", no_ground_topic, "/points_no_ground");
    ROS_INFO("No Ground Output Point Cloud no_ground_point_topic: %s", no_ground_topic.c_str());
    node_handle_.param<std::string>("ground_point_topic", ground_topic, "/points_ground");
    ROS_INFO("Only Ground Output Point Cloud ground_topic: %s", ground_topic.c_str());

    point_node_sub_ = node_handle_.subscribe(input_point_topic_, 10, &Ray_ground_filter::CloudCallback, this);

    //发布滤除地面后的点云和地面点云
    groundless_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 2);
    ground_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(ground_topic, 2);

    dsrv_ = new dynamic_reconfigure::Server<ray_ground_filter::RayConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<ray_ground_filter::RayConfig>::CallbackType cb = boost::bind(&Ray_ground_filter::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    ROS_INFO("clipping_height[deg2]: %f", clipping_height_);

    ros::spin();
}
