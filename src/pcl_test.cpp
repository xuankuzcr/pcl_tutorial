
#include <ros/ros.h>
#include<iostream> 
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件
#include <pcl/visualization/pcl_visualizer.h>
 
 /*
ros::Publisher pub;   //ros发布话题并命名
 
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
 
  // Do data processing here...
  output = *input;
 
  // Publish the data.
  pub.publish (output);
}
 */
//using namespace std;
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "test_pcl");
  ros::NodeHandle nh;
 
  // Create a ROS subscriber for the input point cloud
 // ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
 
  // Create a ROS publisher for the output point cloud
 // pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
 
  // Spin
  //ros::spin();

  if(argc != 2)
    {
        std::cerr << std::endl << "Usage: rosrun pcl_tutorials pcl_test name.pcd" << std::endl;
        return 1;
    }

  char* canshu = argv[1];
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::io::loadPCDFile (canshu, *cloud);

  // 创建 KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // 定义最小二乘实现的对象mls
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
  mls.setComputeNormals (true);  //设置在最小二乘计算中需要进行法线估计

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (1.0);

  // Reconstruct
  mls.process (mls_points);

  // Save outputmls_points
  pcl::io::savePCDFile ("horizon-mls.pcd", mls_points);
  system("pcl_viewer horizon-mls.pcd horizon.pcd");


 /*
  while(!viewer->wasStopped()){
        viewer->spinOnce();
    }
*/
  return 0;

}
