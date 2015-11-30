/*
 * TrajectoryControlActionServer.h
 *
 *  Created on: Jan 13, 2014
 *      Author: alcor
 */

#ifndef TRAJECTORYCONTROLACTIONSERVER_H_
#define TRAJECTORYCONTROLACTIONSERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ms_trajectory_control_msgs/TrajectoryControlAction.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nifti_robot_driver_msgs/Tracks.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <actionlib/client/simple_action_client.h>

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
	T v;
	if (n.getParam(name, v))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}


class TrajectoryControlActionServer
{
protected:
	ros::NodeHandle node;
	ros::NodeHandle n_;
	std::string action_name;
	actionlib::SimpleActionServer<ms_trajectory_control_msgs::TrajectoryControlAction> as;
	actionlib::SimpleActionClient<ms_trajectory_control_msgs::TrajectoryControlAction> ac;

	ms_trajectory_control_msgs::TrajectoryControlFeedback feedback;
	ms_trajectory_control_msgs::TrajectoryControlResult result;

	std::string odom_frame_id;
	std::string global_frame_id;
	std::string robot_frame_id;

	std::string fl_frame_id;
	std::string fr_frame_id;
	std::string rl_frame_id;
	std::string rr_frame_id;
	std::string imu_frame_id;

	double displacement;
	double vel_reference;
	std::vector<double> tip_over_axes_coeff;

	tf::TransformListener tf_;

	std::string imu_odom_topic;
	ros::Subscriber imu_odom_sub;

	std::string tracks_vel_cmd_topic;
	ros::Publisher tracks_vel_cmd_pub;

	tf::StampedTransform real_robot_pose_map;
	tf::StampedTransform real_robot_poseB_map;
	tf::StampedTransform current_real_robot_pose_odom;
	tf::StampedTransform from_odom_to_map;

	tf::StampedTransform frontLeftF;
	tf::StampedTransform frontRightF;
	tf::StampedTransform rearLeftF;
	tf::StampedTransform rearRightF;
	tf::StampedTransform imu_t;

	double linear_vel;
	double angular_vel;
	double robot_width;
	double k1;
	double k2;
	double delay;

	std::string global_path_topic;
	ros::Publisher global_path_;

	std::string local_path_topic;
	ros::Publisher local_path_;

	nav_msgs::Path global_path;
	nav_msgs::Path local_path;
	nav_msgs::Path global_plan;

	std::string cmd_topic;
	ros::Publisher cmd_pub;

	std::string imu_topic;
	ros::Subscriber imu_sub;

	std::string robot_path_topic;


 ros::Subscriber nav_path_sub;


public:
	TrajectoryControlActionServer(std::string);
	~TrajectoryControlActionServer();
	void odomMsgToStampedTransform(nav_msgs::Odometry,tf::StampedTransform&);
	void realRobotPoseB(double,tf::StampedTransform,tf::StampedTransform&);
	bool getRobotPose(tf::StampedTransform&,tf::StampedTransform&,tf::StampedTransform&);
	bool getRobotPose2(tf::StampedTransform&,tf::StampedTransform&,tf::StampedTransform&,tf::StampedTransform&,tf::StampedTransform&,tf::StampedTransform&,tf::StampedTransform&,tf::StampedTransform&);
	void filtering(nav_msgs::Path,nav_msgs::Path&);
	void buildUserDefinedTrajectory(double,geometry_msgs::PoseStamped,geometry_msgs::Pose&,geometry_msgs::Twist&);
	void getRobotCommands(double, tf::StampedTransform, double, double, geometry_msgs::Pose, geometry_msgs::Twist, double&, double&);
	void getTracksVelCmd(double,double,double,nifti_robot_driver_msgs::Tracks&);
	void imuOdomCallback(const nav_msgs::OdometryConstPtr&);
	void executeCallback(const ms_trajectory_control_msgs::TrajectoryControlGoalConstPtr&);
	void tipOverAxis(tf::StampedTransform&,tf::StampedTransform&,tf::StampedTransform&,tf::StampedTransform&,std::vector<double>&);
	void imuDataCallback(const sensor_msgs::ImuConstPtr&);
	

	void pathCallBack(const nav_msgs::PathConstPtr&);
};


#endif /* TRAJECTORYCONTROLACTIONSERVER_H_ */
