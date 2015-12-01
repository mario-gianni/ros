#include <ros/ros.h>
#include <ms_path_planner/NormalEstimationPcl.h>
#include <ms_path_planner/ColorNormalsPcl.h>
#include <ms_path_planner/ConversionPcl.h>
#include <ms_path_planner/MakeNormalsMarkers.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>

NormalEstimationPcl<pcl::PointXYZRGBNormal> normal;
ConversionPcl<pcl::PointXYZ> convpcl;

ros::Publisher pcl_normal_pub;
ros::Publisher marker_normal_pub;

void visualizeNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_norm)
{
	geometry_msgs::PoseArray poseArray;
	makeNormalsMarkers(pcl_norm, poseArray);
	ROS_INFO("normals poseArray size: %d",poseArray.poses.size());
	marker_normal_pub.publish(poseArray);
}

void pointCloudCallback(const sensor_msgs::PointCloud2& scan_msg)
{
	ROS_INFO("Received a new PointCloud2 message");

	NormalEstimationPclConfig normal_config = normal.getConfig();

	pcl::PointCloud<pcl::PointXYZ> scan_pcl;
	pcl::PointCloud<pcl::PointXYZRGBNormal> scan_norm_pcl;
	convpcl.transform(scan_msg, scan_pcl);
	pcl::copyPointCloud(scan_pcl, scan_norm_pcl);

	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> scan_norm_kdtree;
	scan_norm_kdtree.setInputCloud(scan_norm_pcl.makeShared());

	tf::Transform t;
	pcl::PointXYZ laser_center;
	convpcl.getLastTransform(t);
	convpcl.getFrameOrigin(normal_config.laser_frame, laser_center);
	normal.computeNormals(scan_norm_pcl, scan_norm_kdtree, laser_center);

	visualizeNormals(scan_norm_pcl);
	colorNormalsPCL(scan_norm_pcl);

	sensor_msgs::PointCloud2 scan_norm_msg;
	pcl::toROSMsg(scan_norm_pcl, scan_norm_msg);
	pcl_normal_pub.publish(scan_norm_msg);
}

void normalConfigCallback(NormalEstimationPclConfig& config, uint32_t level)
{
	normal.setConfig(config);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_normal");

	tf::TransformListener tf_listener(ros::Duration(10.0));

	ros::NodeHandle n("~");

	dynamic_reconfigure::Server<NormalEstimationPclConfig> normal_config_server(ros::NodeHandle("~/NormalEstimationPcl"));
	normal_config_server.setCallback(boost::bind(&normalConfigCallback, _1, _2));

	convpcl.setTFListener(tf_listener);

	ros::Subscriber sub_scan = n.subscribe("/dynamic_point_cloud", 1, pointCloudCallback);

	pcl_normal_pub = n.advertise<sensor_msgs::PointCloud2>("/pcl_normal", 1, true);
	marker_normal_pub = n.advertise<geometry_msgs::PoseArray>("/normals_marker",1);

	ros::spin();
	return 0;
}

