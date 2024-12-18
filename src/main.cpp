#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//#include <pcl/concatenate.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <cmath>

ros::Publisher pub;
ros::Publisher pub_normals;

void publishNormals(ros::Publisher& pub, pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr indicate_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // 定义旋转矩阵
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

    // 绕 Y 轴旋转 -90 度
    float angle_y = M_PI / 2; // -90 度
    Eigen::Matrix4f rotation_y = Eigen::Matrix4f::Identity();
    rotation_y(0, 0) = cos(angle_y);
    rotation_y(0, 2) = sin(angle_y);
    rotation_y(2, 0) = -sin(angle_y);
    rotation_y(2, 2) = cos(angle_y);

    // 绕 Z 轴旋转 90 度
    float angle_z = -M_PI / 2; // 90 度
    Eigen::Matrix4f rotation_z = Eigen::Matrix4f::Identity();
    rotation_z(0, 0) = cos(angle_z);
    rotation_z(0, 1) = -sin(angle_z);
    rotation_z(1, 0) = sin(angle_z);
    rotation_z(1, 1) = cos(angle_z);

    // 组合变换矩阵
    transformation = rotation_y * rotation_z;

    // 创建一个新的点云用于存储变换后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 进行坐标变换
    pcl::transformPointCloud(*cloud, *transformed_cloud, transformation);

    // 现在 transformed_cloud 包含变换后的点云数据
    *cloud = *transformed_cloud;
}


// 带rgb的点云图回调
void cloud_rgb_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);;
  pcl::fromROSMsg(*cloud_msg,*temp_cloud);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(temp_cloud);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
  sor.filter(*rgb_cloud);
}

// 不带rgb的纯点云图，相当于主回调
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
//   pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//   pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//   pcl::PCLPointCloud2 cloud_filtered;

  // 保存当前surface搜索点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);;
  pcl::fromROSMsg(*cloud_msg,*temp_cloud);

  // 对点云进行坐标变换修正
  transformPointCloud(temp_cloud);
  // TODO：对surface进行滤波，去除异常值，降采样
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(temp_cloud);
  sor.setLeafSize(0.004f, 0.004f, 0.004f);
  sor.filter(*surface_cloud);

  /**********特征点云处理：筛选出相机中心点**********/
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (temp_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.01, 0.01);
  //pass.setNegative (true);
  pass.filter (*indicate_cloud);

  pass.setInputCloud (indicate_cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits (-0.01, 0.01);
  pass.filter (*indicate_cloud);

  sor.setInputCloud(indicate_cloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*indicate_cloud);
  /*********************************************/

  /**************特征点云处理：降采样**************/
  // // 降采样提取indicate特征点云
  // sor.setInputCloud(temp_cloud);
  // sor.setLeafSize(0.05f, 0.05f, 0.05f);
  // sor.filter(*indicate_cloud);
  /*********************************************/


  /********************/
  // 计算法向向量，方法一使用knn搜索并PCA主成分分析
  //pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

  ne.setInputCloud (indicate_cloud);
  ne.setSearchSurface(surface_cloud);
  ne.setSearchMethod (tree);
  ne.setKSearch(25);
  ne.setViewPoint(0.0,0.0,0.0);
  // Use all neighbors in a sphere of radius 3cm
//   ne.setRadiusSearch (0.03);
  // Compute the features
  ne.compute (*cloud_normals);


  //合并点云和法向量
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_rgb_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*indicate_cloud, *cloud_normals, *cloud_with_rgb_normals);

  /**********************/

  /**********************/
  // 计算法向量，方法二使用积分图计算
  // pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // temp_cloud->width = 640;
  // temp_cloud->height = 400;
  // ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  // ne.setMaxDepthChangeFactor(0.02f);
  // ne.setNormalSmoothingSize(10.0f);
  // ne.setInputCloud(temp_cloud);
  // ne.compute(*cloud_normals);

  // //合并点云和法向量
  // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_rgb_normals(new pcl::PointCloud<pcl::PointNormal>);
  // pcl::concatenateFields(*temp_cloud, *cloud_normals, *cloud_with_rgb_normals);

  // pcl::VoxelGrid<pcl::PointNormal> sor_normal;
  // sor_normal.setInputCloud(cloud_with_rgb_normals);
  // sor_normal.setLeafSize(0.04f, 0.04f, 0.04f);
  // sor_normal.filter(*cloud_with_rgb_normals);
  /**********************/

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_with_rgb_normals, output);
  output.header.frame_id = "camera_link"; // 设置坐标系
  output.header.stamp = ros::Time::now(); // 设置时间戳

  // Publish the data
  pub.publish (output);
  publishNormals(pub_normals,cloud_with_rgb_normals);
}

// 发布法向量可视化函数
void publishNormals(ros::Publisher& pub, pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
    visualization_msgs::MarkerArray normals_markers;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "normals";
        marker.id = i; // 每个箭头一个唯一的 ID
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.03);
        marker.scale.x = 0.005; // 箭头基底大小
        marker.scale.y = 0.01; // 箭头头部大小
        marker.color.a = 1.0;   // 透明度
        marker.color.r = 0.0;   // 红色
        marker.color.g = 1.0;   // 绿色
        marker.color.b = 0.0;   // 蓝色

        // 设置起始点和结束点
        marker.points.resize(2);
        marker.points[0].x = cloud->points[i].x;
        marker.points[0].y = cloud->points[i].y;
        marker.points[0].z = cloud->points[i].z;
        marker.points[1].x = cloud->points[i].x + cloud->points[i].normal_x * 0.1; // 法向量长度
        marker.points[1].y = cloud->points[i].y + cloud->points[i].normal_y * 0.1;
        marker.points[1].z = cloud->points[i].z + cloud->points[i].normal_z * 0.1;

        normals_markers.markers.push_back(marker);
    }

    pub.publish(normals_markers); // 发布 MarkerArray
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ROS_INFO("Start normal_detector node");
  ros::init (argc, argv, "normal_detector");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
  // ros::Subscriber sub_cloud_rgb = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_rgb_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("filtered_points", 1);
  pub_normals = nh.advertise<visualization_msgs::MarkerArray> ("normals", 1);

  // Spin
  ros::spin ();
}