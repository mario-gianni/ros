#include <ros/ros.h>
#include <ms_path_planner/DynamicJoinPcl.h>
#include <ms_path_planner/ClusterPcl.h>
#include <ms_path_planner/ConversionPcl.h>
#include <ms_path_planner/NormalEstimationPcl.h>
#include <ms_path_planner/ColorNormalsPcl.h>
#include <ms_path_planner/MakeNormalsMarkers.h>
#include <ms_path_planner/TravAnal.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>

DynamicJoinPcl<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> dynjoinpcl;
NormalEstimationPcl<pcl::PointXYZRGBNormal> normal;
ConversionPcl<pcl::PointXYZ> convpcl;
ClusterPcl<pcl::PointXYZRGBNormal> clusteringpcl;
TravAnal trav;

ros::Publisher pcl_normal_pub;
ros::Publisher marker_normal_pub;
ros::Publisher pcl_pub_dyn;
ros::Publisher pcl_pub_nowall;
ros::Publisher pcl_pub_wall;
ros::Publisher pcl_pub_borders;
ros::Publisher pcl_pub_segmented;
ros::Publisher pcl_pub_traversability;

ros::Publisher pcl_pub_clearence;
ros::Publisher pcl_pub_density;
ros::Publisher pcl_pub_label;
ros::Publisher pcl_pub_roughness;

pcl::PointCloud<pcl::PointXYZRGBNormal> map_pcl, nowall_pcl, border_pcl, segmented_pcl, wall_pcl;

void visualizeNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_norm)
{
	geometry_msgs::PoseArray poseArray;
	makeNormalsMarkers(pcl_norm, poseArray);
	ROS_INFO("normals poseArray size: %d",poseArray.poses.size());
	marker_normal_pub.publish(poseArray);
}

void pointCloudCallback(const sensor_msgs::PointCloud2& scan_msg) {

	ROS_INFO("Received a new PointCloud2 message");

	DynamicJoinPclConfig dynjoinpcl_config = dynjoinpcl.getConfig();

	// normal estimation

	pcl::PointCloud<pcl::PointXYZ> scan_pcl;
	pcl::PointCloud<pcl::PointXYZRGBNormal> scan_norm_pcl;
	convpcl.transform(scan_msg, scan_pcl);
	pcl::copyPointCloud(scan_pcl, scan_norm_pcl);

	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> scan_norm_kdtree;
	scan_norm_kdtree.setInputCloud(scan_norm_pcl.makeShared());

	tf::Transform t;
	pcl::PointXYZ laser_center;
	convpcl.getLastTransform(t);
	convpcl.getFrameOrigin(dynjoinpcl_config.laser_frame, laser_center);
	normal.computeNormals(scan_norm_pcl, scan_norm_kdtree, laser_center);

	sensor_msgs::PointCloud2 scan_norm_msg;
	pcl::toROSMsg(scan_norm_pcl, scan_norm_msg);
	pcl_normal_pub.publish(scan_norm_msg);

	// dynamic join

	pcl::PointCloud<pcl::PointXYZRGBNormal> map_new_pcl;

	map_new_pcl.header = map_pcl.header;

	dynjoinpcl.joinPCL(scan_norm_pcl, map_pcl, map_new_pcl, laser_center);
	map_pcl.swap(map_new_pcl);

	//colorNormalsPCL(map_pcl);
	visualizeNormals(map_pcl);

	sensor_msgs::PointCloud2 map_msg_out;
	pcl::toROSMsg(map_pcl, map_msg_out);

	pcl_pub_dyn.publish(map_msg_out);

	if (map_pcl.size() > 0){

		pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> input_kdtree;
		input_kdtree.setInputCloud(map_pcl.makeShared());
		std::vector<ClusterInfo> cluster_info;
		pcl::PointCloud<pcl::PointXYZI> traversability_pcl, clearence_pcl, density_pcl, label_pcl, roughness_pcl;

		clusteringpcl.set_input_pcl(map_pcl);
		clusteringpcl.set_kdtree(input_kdtree);
		clusteringpcl.clustering(nowall_pcl, border_pcl, segmented_pcl,wall_pcl);
		clusteringpcl.get_cluster_info(cluster_info);
		cout<<"clusters_info size: "<<cluster_info.size()<<endl;

		nowall_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		wall_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		border_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		segmented_pcl.header.frame_id = dynjoinpcl_config.global_frame;


		pcl_pub_nowall.publish(nowall_pcl);
		pcl_pub_wall.publish(wall_pcl);
		pcl_pub_segmented.publish(segmented_pcl);

		pcl::PointCloud<pcl::PointXYZRGBNormal> segmented_filtered;
		pcl::VoxelGrid<pcl::PointXYZRGBNormal> sor;
		sor.setInputCloud(segmented_pcl.makeShared());
		sor.setLeafSize(0.05, 0.05, 0.05);
		sor.filter(segmented_filtered);


		trav.set_input(cluster_info, wall_pcl, segmented_pcl);
		trav.computeTrav(traversability_pcl);
		trav.get_pcl(clearence_pcl, density_pcl, label_pcl, roughness_pcl);

		traversability_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		density_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		label_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		roughness_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		clearence_pcl.header.frame_id = dynjoinpcl_config.global_frame;


		sensor_msgs::PointCloud2 trav_msg_out;
		pcl::toROSMsg(traversability_pcl, trav_msg_out);
		pcl_pub_traversability.publish(trav_msg_out);

		pcl::toROSMsg(clearence_pcl, trav_msg_out);
		pcl_pub_clearence.publish(trav_msg_out);

		pcl::toROSMsg(roughness_pcl, trav_msg_out);
		pcl_pub_roughness.publish(trav_msg_out);

		pcl::toROSMsg(label_pcl, trav_msg_out);
		pcl_pub_label.publish(trav_msg_out);

		pcl::toROSMsg(density_pcl, trav_msg_out);
		pcl_pub_density.publish(trav_msg_out);
	}
}

void dynjoinpclConfigCallback(DynamicJoinPclConfig& config, uint32_t level)
{	ROS_INFO("received leafSize: %f",config.leaf_size);
	dynjoinpcl.setConfig(config);
	convpcl.setOutputFrame(config.global_frame);
	map_pcl.header.frame_id = config.global_frame;
}

void normalConfigCallback(NormalEstimationPclConfig& config, uint32_t level)
{
	normal.setConfig(config);
}

void clusteringpclConfigCallback(ClusterPclConfig& config, uint32_t level)
{
	clusteringpcl.setConfig(config);
}

void travConfigCallback(TravAnalConfig& config, uint32_t level)
{
	trav.setConfig(config);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_cluster");

	tf::TransformListener tf_listener(ros::Duration(10.0));

	ros::NodeHandle n("~");

	dynamic_reconfigure::Server<DynamicJoinPclConfig> dynjoinpcl_config_server(ros::NodeHandle("~/DynamicJoinPcl"));
	dynjoinpcl_config_server.setCallback(boost::bind(&dynjoinpclConfigCallback, _1, _2));

	dynamic_reconfigure::Server<NormalEstimationPclConfig> normal_config_server(ros::NodeHandle("~/NormalEstimationPcl"));
	normal_config_server.setCallback(boost::bind(&normalConfigCallback, _1, _2));

	dynamic_reconfigure::Server<ClusterPclConfig> clusteringpcl_config_server(ros::NodeHandle("~/ClusteringPcl"));
	clusteringpcl_config_server.setCallback(boost::bind(&clusteringpclConfigCallback, _1, _2));

	dynamic_reconfigure::Server<TravAnalConfig> trav_config_server(ros::NodeHandle("~/TravAnal"));
	trav_config_server.setCallback(boost::bind(&travConfigCallback, _1, _2));

	convpcl.setTFListener(tf_listener);

	ros::Subscriber sub_pcl = n.subscribe("/dynamic_point_cloud", 1, pointCloudCallback);

	pcl_pub_dyn = n.advertise<sensor_msgs::PointCloud2>("/dynjoinpcl", 1, true);
	pcl_pub_nowall = n.advertise<sensor_msgs::PointCloud2>("/clustered_pcl/no_wall", 1, true);
	pcl_pub_wall = n.advertise<sensor_msgs::PointCloud2>("/clustered_pcl/wall", 1, true);

	pcl_pub_traversability = n.advertise<sensor_msgs::PointCloud2>("/trav/traversability", 1, true);
	pcl_pub_clearence = n.advertise<sensor_msgs::PointCloud2>("/trav/clearence", 1, true);
	pcl_pub_density = n.advertise<sensor_msgs::PointCloud2>("/trav/density", 1, true);
	pcl_pub_label = n.advertise<sensor_msgs::PointCloud2>("/trav/label", 1, true);
	pcl_pub_roughness = n.advertise<sensor_msgs::PointCloud2>("/trav/roughness", 1, true);

	pcl_pub_segmented = n.advertise<sensor_msgs::PointCloud2>("/clustered_pcl/segmented", 1, true);
	pcl_normal_pub = n.advertise<sensor_msgs::PointCloud2>("/normals_pcl", 1, true);
	marker_normal_pub = n.advertise<geometry_msgs::PoseArray>("/normals_marker",1);

	ros::spin();
	return 0;
}
