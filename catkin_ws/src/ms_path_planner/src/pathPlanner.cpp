#include <ros/ros.h>

#include <ms_path_planner/PathPlanning.h>
#include <ms_path_planner/MarkerController.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/thread.hpp>

std::string robot_frame;
std::string goal_topic_name;
std::string int_server_name;
std::string marker_name;

bool wall_flag = false;


pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> wall_kdTree;
pcl::PointCloud<pcl::PointXYZRGBNormal> wall_pcl;
PathPlanning* planner;
ros::Publisher path_pub;
tf::TransformListener *tf_;

bool goal_selection = false;

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
	T v;
	if (n.getParam(name, v))
	{
		//ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}

bool getRobotPose(tf::StampedTransform& robot_pose){

	if (tf_->waitForTransform("/map", robot_frame, ros::Time(),
			ros::Duration(1.0))) {
		try {
			tf_->lookupTransform("/map", robot_frame, ros::Time(),
					robot_pose);
		} catch (tf::LookupException& ex) {
			ROS_INFO(
					"No Transform available Error looking up robot pose: %s\n", ex.what());
			return false;
		} catch (tf::ConnectivityException& ex) {
			ROS_INFO(
					"Connectivity Error looking up robot pose: %s\n", ex.what());
			return false;
		} catch (tf::ExtrapolationException& ex) {
			ROS_INFO(
					"Extrapolation Error looking up robot pose: %s\n", ex.what());
			return false;
		}

		return true;
	} else {
		ROS_INFO("Transformation is not available");
		return false;
	}
}


void pointCloudCallback(const sensor_msgs::PointCloud2& traversability_msg) {

	pcl::PointCloud<pcl::PointXYZI> traversability_pcl;
	pcl::fromROSMsg(traversability_msg, traversability_pcl);
	//ROS_INFO("path planner input set");
	if (traversability_pcl.size() > 0 && wall_flag){

		tf::StampedTransform robot_pose;
		getRobotPose(robot_pose);
		pcl::PointXYZI robot;
		robot.x = robot_pose.getOrigin().x();
		robot.y = robot_pose.getOrigin().y();
		robot.z = robot_pose.getOrigin().z();

		pcl::KdTreeFLANN<pcl::PointXYZI> traversability_kdtree;
		traversability_kdtree.setInputCloud(traversability_pcl.makeShared());

		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		traversability_kdtree.nearestKSearch(robot, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
		planner->set_input(traversability_pcl, wall_pcl, wall_kdTree, traversability_kdtree, pointIdxNKNSearch[0]);
		wall_flag = false;
		if (goal_selection){

			nav_msgs::Path path;

			//ROS_INFO("compute path");
			if(goal_selection){
				if(planner->planning(path)){
					path_pub.publish(path);

				}
				else{
					//ROS_INFO("no path exist for desired goal, please choose another goal");
					goal_selection = true;
				}
				//ROS_INFO("path_computed");
			}
		}
	}
}

void wallCallback(const sensor_msgs::PointCloud2& wall_msg){
	pcl::fromROSMsg(wall_msg, wall_pcl);
	wall_kdTree.setInputCloud(wall_pcl.makeShared());
	wall_flag = true;
}

void goalSelectionCallback(geometry_msgs::PoseStamped goal_){
	pcl::PointXYZI  p;
	p.x = goal_.pose.position.x;
	p.y = goal_.pose.position.y;
	p.z = goal_.pose.position.z;
	planner->set_goal(p);
	goal_selection = true;
	//ROS_INFO("goal selection");
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_cluster");


	tf::TransformListener tf;
	tf_ = &tf;

	tf::TransformListener tf_listener(ros::Duration(10.0));

	ros::NodeHandle n("~");

	robot_frame = getParam<std::string>(n, "robot_frame", "/base_link");
	goal_topic_name = getParam<std::string>(n, "goal_topic_name", "/goal_topic");
	int_server_name = getParam<std::string>(n, "int_server_name", "marker_controller");
	marker_name = getParam<std::string>(n, "int_marker_name", "ugv");

	PathPlanning new_planner(n);
	planner = &new_planner;

	ros::Subscriber sub_trav = n.subscribe("/trav/traversability", 1, pointCloudCallback);
	ros::Subscriber sub_wall = n.subscribe("/clustered_pcl/wall", 1, wallCallback);
	ros::Subscriber sub_goal = n.subscribe(goal_topic_name, 1, goalSelectionCallback);

	path_pub = n.advertise<nav_msgs::Path>("/robot_path",1);

	MarkerController marker(goal_topic_name,int_server_name,marker_name);
	ros::spin();

	marker.reset();
	return 0;
}
