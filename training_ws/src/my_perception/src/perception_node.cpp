#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> //hydro

// Include ROS SRV files
#include <my_perception/partUpdate.h>

#include <geometry_msgs/Pose.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> //hydro
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/common/centroid.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>



std::vector<ros::Publisher> pub_vec;
std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;



bool partUpdate (my_perception::partUpdate::Request &req,
                     my_perception::partUpdate::Response &res) {
  
  ros::NodeHandle n;
  ros::NodeHandle priv_n_("~");
  
  /*
   * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
   */

  std::string cloud_topic, world_frame, camera_frame;

  float voxel_leaf_size;
  float x_filter_min, x_filter_max, y_filter_min, y_filter_max, z_filter_min, z_filter_max;
  int plane_max_iter;
  float plane_dist_thresh;
  float cluster_tol;
  int cluster_min_size;
  int cluster_max_size;

  cloud_topic = priv_n_.param<std::string>("cloud_topic", "/camera/depth/color/points");
  world_frame = priv_n_.param<std::string>("world_frame", "prbt_base");
  camera_frame = priv_n_.param<std::string>("camera_frame", "camera_depth_frame");
  voxel_leaf_size = priv_n_.param<float>("voxel_leaf_size", 0.002);
  x_filter_min = priv_n_.param<float>("x_filter_min", -2.5);
  x_filter_max = priv_n_.param<float>("x_filter_max",  2.5);
  y_filter_min = priv_n_.param<float>("y_filter_min", -2.5);
  y_filter_max = priv_n_.param<float>("y_filter_max",  2.5);
  z_filter_min = priv_n_.param<float>("z_filter_min", -2.5);
  z_filter_max = priv_n_.param<float>("z_filter_max",  2.5);
  plane_max_iter = priv_n_.param<int>("plane_max_iterations", 50);
  plane_dist_thresh = priv_n_.param<float>("plane_distance_threshold", 0.05);
  cluster_tol = priv_n_.param<float>("cluster_tolerance", 0.01);
  cluster_min_size = priv_n_.param<int>("cluster_min_size", 100);
  cluster_max_size = priv_n_.param<int>("cluster_max_size", 50000);    

  /*
   * LISTEN FOR POINTCLOUD
   */
  std::string topic = n.resolveName(cloud_topic);
  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic "<< topic);
  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
               ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, n);

  /*
   * TRANSFORM POINTCLOUD FROM CAMERA FRAME TO WORLD FRAME
   */
  tf::TransformListener listener;
  tf::StampedTransform stransform;
  try
  {
    listener.waitForTransform(world_frame, recent_cloud->header.frame_id,  ros::Time::now(), ros::Duration(6.0));
    listener.lookupTransform(world_frame, recent_cloud->header.frame_id,  ros::Time(0), stransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  sensor_msgs::PointCloud2 transformed_cloud;
//  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
//               ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);
  pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);


  /*
   * CONVERT POINTCLOUD ROS->PCL
   */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (transformed_cloud, cloud);


  /* ========================================
   * Fill Code: VOXEL GRID
   * ========================================*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> (cloud));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud (cloud_ptr);
  voxel_filter.setLeafSize (voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_filter.filter (*cloud_voxel_filtered);

  //ROS_INFO_STREAM("Original cloud  had " << cloud_ptr->size() << " points");
  //ROS_INFO_STREAM("Downsampled cloud  with " << cloud_voxel_filtered->size() << " points");

  /* ========================================
   * Fill Code: PASSTHROUGH FILTER(S)
   * ========================================*/
  pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(cloud_voxel_filtered);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(x_filter_min, x_filter_max);
  pass_x.filter(xf_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(xf_cloud_ptr);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(y_filter_min, y_filter_max);
  pass_y.filter(yf_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(yf_cloud_ptr);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(z_filter_min, z_filter_max);
  pass_z.filter(zf_cloud);

  /* ========================================
   * Fill Code: CROPBOX (OPTIONAL)
   * ========================================*/
  pcl::PointCloud<pcl::PointXYZ> xyz_filtered_cloud;
  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setInputCloud(cloud_voxel_filtered);
  Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
  Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
  crop.setMin(min_point);
  crop.setMax(max_point);
  crop.filter(xyz_filtered_cloud);

  /* ========================================
   * Fill Code: PLANE SEGEMENTATION
   * ========================================*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(xyz_filtered_cloud));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (plane_max_iter);
  seg.setDistanceThreshold (plane_dist_thresh);
  // Segment the largest planar component from the cropped cloud
  seg.setInputCloud (cropped_cloud);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;
    //break;
  }

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cropped_cloud);
  extract.setIndices(inliers);
  extract.setNegative (false);

  // Get the points associated with the planar surface
  extract.filter (*cloud_plane);
  ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." );

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_f);


 /* =====================================
  *        EL NOSTRE PUBLISHER
  * ======================================*/

//euclidean cluster extraction 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  *cloud_filtered = *cloud_f;
  tree->setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tol);
  ec.setMinClusterSize(cluster_min_size);
  ec.setMaxClusterSize(cluster_max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

//Create a publisher for each cluster
  for (int i=0; i<cluster_indices.size(); ++i) {
    std::string topicName = "/cluster_" + boost::lexical_cast<std::string>(i);
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> (topicName, 1);
    pub_vec.push_back(pub);
  }

  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;

  int j = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    //std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
    clusters.push_back(cloud_cluster);
  
    sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
    pc2_clusters.push_back(tempROSMsg);
    tempROSMsg->header.frame_id = world_frame;
    tempROSMsg-> header.stamp = ros::Time::now();
    pub_vec[j].publish(tempROSMsg);
    ++j;
  }

  for(int i=0; i<j; i++){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_vec = clusters.at(i);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster_vec, centroid);
    Eigen::Vector3f vec3 = centroid.head<3>();

    //Broadcast transform
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "prbt_base";
    static_transformStamped.child_frame_id = "cluster_0";

    static_transformStamped.transform.translation.x = vec3(0);
    static_transformStamped.transform.translation.y = vec3(1);
    static_transformStamped.transform.translation.z = vec3(2);

    tf2::Quaternion q;
    q.setRPY(0 ,3.1415, 0);

    static_transformStamped.transform.rotation.x = q.x();
    static_transformStamped.transform.rotation.y = q.y();
    static_transformStamped.transform.rotation.z = q.z();
    static_transformStamped.transform.rotation.w = q.w();

    static_broadcaster.sendTransform(static_transformStamped);
  }
    res.updated = true;
    return true; 
}

int main(int argc, char *argv[])
{
  /*
   * INITIALIZE ROS NODE
   */
  ros::init(argc, argv, "perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");
  ros::ServiceServer service = nh.advertiseService("partUpdate", partUpdate);
  ROS_INFO("Perception Service Advertised.");
  ros::spin();

  return 0;
}

