#include <ros/ros.h>
#include <ms_path_planner/DynamicJoinPcl.h>
#include <ms_path_planner/NormalEstimationPcl.h>
#include <ms_path_planner/ColorNormalsPcl.h>
#include <ms_path_planner/ConversionPcl.h>
#include <ms_path_planner/MakeNormalsMarkers.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>

DynamicJoinPcl<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> dynjoinpcl;
NormalEstimationPcl<pcl::PointXYZRGBNormal> normal;
ConversionPcl<pcl::PointXYZ> convpcl;

ros::Publisher pcl_normal_pub;
ros::Publisher marker_normal_pub;
ros::Publisher pcl_pub;

pcl::PointCloud<pcl::PointXYZRGBNormal> map_pcl;

std::string laser_frame;
std::string global_frame;

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

	DynamicJoinPclConfig dynjoinpcl_config = dynjoinpcl.getConfig();
	NormalEstimationPclConfig normal_config = normal.getConfig();

	// downsample
	pcl::PointCloud<pcl::PointXYZ> scan_pcl;
	pcl::PointCloud<pcl::PointXYZRGBNormal> scan_norm_pcl;
	convpcl.transform(scan_msg, scan_pcl);
	scan_norm_pcl.header = scan_pcl.header;
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(scan_pcl.makeShared());
	sor.setLeafSize(dynjoinpcl_config.leaf_size, dynjoinpcl_config.leaf_size, dynjoinpcl_config.leaf_size);
	sor.filter(scan_pcl);
	pcl::copyPointCloud(scan_pcl, scan_norm_pcl);

	// normal estimation
	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> scan_norm_kdtree;
	scan_norm_kdtree.setInputCloud(scan_norm_pcl.makeShared());
	tf::Transform t;
	pcl::PointXYZ laser_center;
	convpcl.getLastTransform(t);
	convpcl.getFrameOrigin(dynjoinpcl_config.laser_frame, laser_center);
	normal.computeNormals(scan_norm_pcl, scan_norm_kdtree, laser_center);

	// dynamic join

	pcl::PointCloud<pcl::PointXYZRGBNormal> map_new_pcl;
	map_new_pcl.header = map_pcl.header;
	dynjoinpcl.joinPCL(scan_norm_pcl, map_pcl, map_new_pcl, laser_center);
	map_pcl.swap(map_new_pcl);
	//colorNormalsPCL(map_pcl);
	visualizeNormals(map_pcl);
	sensor_msgs::PointCloud2 map_msg_out;
	pcl::toROSMsg(map_pcl, map_msg_out);
	pcl_pub.publish(map_msg_out);
}

void dynjoinpclConfigCallback(DynamicJoinPclConfig& config, uint32_t level)
{
	dynjoinpcl.setConfig(config);
	convpcl.setOutputFrame(config.global_frame);
	map_pcl.header.frame_id = config.global_frame;
}

void normalConfigCallback(NormalEstimationPclConfig& config, uint32_t level)
{
	normal.setConfig(config);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_dynjoinpcl_with_normal");

	tf::TransformListener tf_listener(ros::Duration(10.0));

	ros::NodeHandle n("~");

	dynamic_reconfigure::Server<DynamicJoinPclConfig> dynjoinpcl_config_server(ros::NodeHandle("~/DynamicJoinPcl"));
	dynjoinpcl_config_server.setCallback(boost::bind(&dynjoinpclConfigCallback, _1, _2));

	dynamic_reconfigure::Server<NormalEstimationPclConfig> normal_config_server(ros::NodeHandle("~/NormalEstimationPcl"));
	normal_config_server.setCallback(boost::bind(&normalConfigCallback, _1, _2));

    convpcl.setTFListener(tf_listener);

	ros::Subscriber sub_pcl = n.subscribe("/dynamic_point_cloud", 1, pointCloudCallback);

	marker_normal_pub = n.advertise<geometry_msgs::PoseArray>("/normals_marker",1);
	pcl_pub = n.advertise<sensor_msgs::PointCloud2>("/dynjoinpcl", 1, true);

	ros::spin();
	return 0;
}

