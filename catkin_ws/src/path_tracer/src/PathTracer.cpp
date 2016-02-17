#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
	T v;
	if (n.getParam(name, v))
	{
		ROS_INFO_STREAM("[NLA] Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
		ROS_WARN_STREAM("[NLA] Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}

class PathTracer
{
	public:
		PathTracer();
		virtual ~PathTracer();

	protected:

	//! public NodeHandle
	ros::NodeHandle n;

	//! private NodeHandle
	ros::NodeHandle n_;

	tf::TransformListener tf_listener;

	ros::Subscriber pose_sub;
	
	ros::Publisher path_pub;

	std::string pose_in;

	std::string path_out;

	std::string path_frame;

	nav_msgs::Path path;

	void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

};


PathTracer::PathTracer():
n_("~"),
tf_listener(ros::Duration(60.))
{
	pose_in = getParam<std::string>(n_, "pose_in", "/current_pose");
	path_out = getParam<std::string>(n_, "path_out", "/path_followed");
	path_frame = getParam<std::string>(n_, "path_frame", "/map");
	pose_sub = n.subscribe(pose_in, 50, &PathTracer::pose_cb, this);
	path_pub = n.advertise<nav_msgs::Path>(path_out,50);
	path.header.stamp = ros::Time::now();
	path.header.frame_id = path_frame;
	path.poses.clear();
}

PathTracer::~PathTracer()
{
	// Nothing to do?
}

void PathTracer::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
	std::string current_frame = msg->header.frame_id;
	geometry_msgs::PoseStamped p;

	if(current_frame.compare(path_frame) == 0)
	{		
		p.header.frame_id = msg->header.frame_id;
		p.header.stamp = msg->header.stamp;
		p.pose.position.x = msg->pose.position.x;
		p.pose.position.y = msg->pose.position.y;
		p.pose.position.z = msg->pose.position.z;
		p.pose.orientation.x = msg->pose.orientation.x;
		p.pose.orientation.y = msg->pose.orientation.y;
		p.pose.orientation.z = msg->pose.orientation.z;
		p.pose.orientation.w = msg->pose.orientation.w;
		path.poses.push_back(p);
	}
	else
	{
		try
		{
  			tf_listener.transformPose(current_frame,*msg,p);
			path.poses.push_back(p);
		}
  		catch( tf::TransformException ex)
  		{
      			ROS_ERROR("transfrom exception : %s",ex.what());
  		}

	}

	path_pub.publish(path);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, ros::this_node::getName());

	PathTracer tracer;

	ros::spin();

	return 0;
}

