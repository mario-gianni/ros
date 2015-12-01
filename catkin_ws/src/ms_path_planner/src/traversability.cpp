#include <ros/ros.h>
#include <ms_path_planner/DynamicJoinPcl.h>
#include <ms_path_planner/ClusterPcl.h>
#include <ms_path_planner/ConversionPcl.h>
#include <ms_path_planner/ColorNormalsPcl.h>
#include <ms_path_planner/MakeNormalsMarkers.h>
#include <ms_path_planner/TravAnal.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>

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

void pointCloudCallback(const sensor_msgs::PointCloud2& map) {

	pcl::fromROSMsg(map, map_pcl);
	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> map_norm_kdtree;
	map_norm_kdtree.setInputCloud(map_pcl.makeShared());

	if (map_pcl.size() > 0){


		std::vector<int> cluster_info;
		pcl::PointCloud<pcl::PointXYZI> traversability_pcl, clearence_pcl, density_pcl, label_pcl, roughness_pcl;

		clusteringpcl.set_input_pcl(map_pcl);
		clusteringpcl.set_kdtree(map_norm_kdtree);
		clusteringpcl.clustering(nowall_pcl, border_pcl, segmented_pcl,wall_pcl);
		clusteringpcl.get_cluster_info(cluster_info);

		nowall_pcl.header.frame_id = map.header.frame_id;
		wall_pcl.header.frame_id = map.header.frame_id;
		border_pcl.header.frame_id = map.header.frame_id;
		segmented_pcl.header.frame_id = map.header.frame_id;

		sensor_msgs::PointCloud2 nowall_msg, wall_msg, segmented_msg;

		pcl::toROSMsg(nowall_pcl, nowall_msg);
		pcl::toROSMsg(wall_pcl, wall_msg);
		pcl::toROSMsg(segmented_pcl, segmented_msg);


		pcl_pub_nowall.publish(nowall_msg);
		pcl_pub_wall.publish(wall_msg);
		pcl_pub_segmented.publish(segmented_msg);

		trav.set_input(cluster_info, wall_pcl, nowall_pcl);
		trav.computeTrav(traversability_pcl);
		trav.get_pcl(clearence_pcl, density_pcl, label_pcl, roughness_pcl);

		traversability_pcl.header.frame_id = map.header.frame_id;
		density_pcl.header.frame_id = map.header.frame_id;
		label_pcl.header.frame_id = map.header.frame_id;
		roughness_pcl.header.frame_id = map.header.frame_id;
		clearence_pcl.header.frame_id = map.header.frame_id;


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


	dynamic_reconfigure::Server<ClusterPclConfig> clusteringpcl_config_server(ros::NodeHandle("~/ClusteringPcl"));
	clusteringpcl_config_server.setCallback(boost::bind(&clusteringpclConfigCallback, _1, _2));

	dynamic_reconfigure::Server<TravAnalConfig> trav_config_server(ros::NodeHandle("~/TravAnal"));
	trav_config_server.setCallback(boost::bind(&travConfigCallback, _1, _2));

	convpcl.setTFListener(tf_listener);

	ros::Subscriber sub_pcl = n.subscribe("/dynjoinpcl", 1, pointCloudCallback);


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
