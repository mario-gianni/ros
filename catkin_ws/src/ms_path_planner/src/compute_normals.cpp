#include <ros/ros.h>
#include <ms_path_planner/DynamicJoinPcl.h>
#include <ms_path_planner/NormalEstimationPcl.h>
#include <ms_path_planner/ColorNormalsPcl.h>
#include <ms_path_planner/ConversionPcl.h>
#include <ms_path_planner/MakeNormalsMarkers.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>

NormalEstimationPcl<pcl::PointXYZRGBNormal> normal;
ConversionPcl<pcl::PointXYZRGBNormal> convpcl;

ros::Publisher marker_normal_pub;
ros::Publisher pcl_pub;

std::string laser_frame;
std::string global_frame;

void visualizeNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_norm)
{
	geometry_msgs::PoseArray poseArray;
	makeNormalsMarkers(pcl_norm, poseArray);
	ROS_INFO("normals poseArray size: %d",poseArray.poses.size());
	marker_normal_pub.publish(poseArray);
}

void pointCloudCallback(const sensor_msgs::PointCloud2& pcl_msg)
{
	ROS_INFO("Received a new PointCloud2 message");

	NormalEstimationPclConfig normal_config = normal.getConfig();

	// downsample
	pcl::PointCloud<pcl::PointXYZRGBNormal> pcl;
        //pcl::PointCloud<pcl::PointXYZRGBNormal> pcl_out;
	convpcl.transform(pcl_msg, pcl);

	// normal estimation
	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
	kdtree.setInputCloud(pcl.makeShared());
	tf::Transform t;
	pcl::PointXYZ laser_center;
	convpcl.getLastTransform(t);
	//std::cout<<"Laser Frame: " << normal_config.laser_frame.c_str() << std::endl;
	convpcl.getFrameOrigin(normal_config.laser_frame, laser_center);
	normal.computeNormals(pcl, kdtree, laser_center);
    	//std::vector<int> index;
    	//pcl::removeNaNNormalsFromPointCloud(pcl,pcl_out,index);
	//ROS_INFO("Num of removed points [%d]",index.size());

	//colorNormalsPCL(map_pcl);
	visualizeNormals(pcl);
	sensor_msgs::PointCloud2 msg_out;
	pcl::toROSMsg(pcl, msg_out);
	pcl_pub.publish(msg_out);
}

void normalConfigCallback(NormalEstimationPclConfig& config, uint32_t level)
{
	normal.setConfig(config);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "compute_normals");

	tf::TransformListener tf_listener(ros::Duration(10.0));

	ros::NodeHandle n("~");

	dynamic_reconfigure::Server<NormalEstimationPclConfig> normal_config_server(ros::NodeHandle("~/NormalEstimationPcl"));
	normal_config_server.setCallback(boost::bind(&normalConfigCallback, _1, _2));

    convpcl.setTFListener(tf_listener);

	ros::Subscriber sub_pcl = n.subscribe("/cloud_in", 1, pointCloudCallback);

	marker_normal_pub = n.advertise<geometry_msgs::PoseArray>("/normals_marker",1);
	pcl_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_out", 1, true);

	ros::spin();
	return 0;
}

