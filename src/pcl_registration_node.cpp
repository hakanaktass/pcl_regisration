#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>

#include <boost/thread.hpp>


using namespace std::chrono_literals;

class PointCloudHandlerNode:public rclcpp::Node{
    public:
      PointCloudHandlerNode();
    private:
      // params
      std::string mode_;
      std::string target_cloud_path_;
      std::string input_cloud_path_;
      std::string output_cloud_path_;

      void run_pcl_registration(bool visualize_over_pcl);
      void read_clouds();
      void performNDTRegistration();
      void publish_clouds();
      void visualize();
      pcl::PointCloud<pcl::PointXYZRGB> color_pointcloud(
        pcl::PointCloud<pcl::PointXYZ> & input_cloud, const double r, const double g, const double b);

      pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud{new pcl::PointCloud<pcl::PointXYZ>};
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud{new pcl::PointCloud<pcl::PointXYZ>};
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud{new pcl::PointCloud<pcl::PointXYZ>};

      sensor_msgs::msg::PointCloud2 pointcloud_msg;

      rclcpp::TimerBase::SharedPtr timer_;

      boost::shared_ptr<boost::thread> visualizer_thread_;

      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_publisher_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_publisher_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_publisher_;
};

PointCloudHandlerNode::PointCloudHandlerNode():Node("pcl_registration")
{
  mode_ = this->declare_parameter("mode", "ROS");
  target_cloud_path_ = this->declare_parameter("target_cloud_path", "src/pcl_registration/data/capture0001.pcd");
  input_cloud_path_ = this->declare_parameter("input_cloud_path", "src/pcl_registration/data/capture0002.pcd");
  output_cloud_path_ = this->declare_parameter("output_cloud_path", "src/pcl_registration/data/transformed.pcd");

  this->source_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/source", 10);
  this->target_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/target", 10);
  this->output_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/output", 10);

  if (mode_ == "ROS") 
  {
    timer_ = this->create_wall_timer(500ms, std::bind(&PointCloudHandlerNode::publish_clouds, this));
    run_pcl_registration(false);
  }
  else 
  {
    run_pcl_registration(true);
  }
}

void PointCloudHandlerNode::run_pcl_registration(bool visualize_over_pcl)
{
  read_clouds();

  performNDTRegistration();

  if (visualize_over_pcl)
  {
    visualize();
  }
} 

void PointCloudHandlerNode::read_clouds()
{
    // Loading first scan of room.
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_cloud_path_, *target_cloud) == -1)
  {
    PCL_ERROR("Couldn't read file (target cloud) \n");
    return;
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;

  // Loading second scan of room from new perspective.
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_cloud_path_, *input_cloud) == -1)
  {
    PCL_ERROR("Couldn't read file (input cloud) \n");
    return;
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;
}

void PointCloudHandlerNode::performNDTRegistration()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

 // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (35);

  // Setting point cloud to be aligned.
  ndt.setInputSource (filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);
// Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  ndt.align (*output_cloud, init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;

            // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("src/pcl_registration/data/transformed.pcd", *output_cloud);
}

void PointCloudHandlerNode::visualize()
{
    // Initializing point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  // Spin Viewer
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spin();
  }
}

pcl::PointCloud<pcl::PointXYZRGB> PointCloudHandlerNode::color_pointcloud(
  pcl::PointCloud<pcl::PointXYZ> & input_cloud, const double r, const double g, const double b)
{
  // Convert point type for coloring
  pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
  pcl::copyPointCloud(input_cloud, colored_cloud);

  // Color points
  for (auto & point : colored_cloud.points)
  {
    point.r = r;
    point.g = g;
    point.b = b;
  }

  return colored_cloud;
}

void PointCloudHandlerNode::publish_clouds()
{
  // Source cloud with red color
  const auto input_colored_cloud = color_pointcloud(*input_cloud, 255.0, 0.0 , 0.0);
  pcl::toROSMsg(input_colored_cloud, pointcloud_msg);
  pointcloud_msg.header.frame_id = "map";
  source_publisher_->publish(pointcloud_msg);

  // Source cloud with red color
  const auto target_colored_cloud = color_pointcloud(*target_cloud, 0.0, 255.0 , 0.0);
  pcl::toROSMsg(target_colored_cloud, pointcloud_msg);
  pointcloud_msg.header.frame_id = "map";
  target_publisher_->publish(pointcloud_msg);

  // Source cloud with red color
  const auto output_colored_cloud = color_pointcloud(*output_cloud, 0.0, 0.0 , 255.0);
  pcl::toROSMsg(output_colored_cloud, pointcloud_msg);
  pointcloud_msg.header.frame_id = "map";
  output_publisher_->publish(pointcloud_msg);
}


int main(int argc, char* argv[]){    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudHandlerNode>());
    rclcpp::shutdown();
    return 0;
}