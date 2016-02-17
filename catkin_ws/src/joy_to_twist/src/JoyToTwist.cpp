#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

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

class JoyToTwist
{
public:
	JoyToTwist();
	virtual ~JoyToTwist();

protected:

	//! public NodeHandle
	ros::NodeHandle n;

	//! private NodeHandle
	ros::NodeHandle n_;

	ros::Subscriber joy_sub;

	ros::Publisher cmd_pub;

	std::string robot_type;

	std::string joy_in;

	std::string cmd_out;

	double init_height;

	double max_height;

	double scale_step;

	double current_height;

	double lin_vel_scale;

	void joy_cb(const sensor_msgs::JoyConstPtr& msg);
};

JoyToTwist::JoyToTwist():
	n_("~")
{
	robot_type = getParam<std::string>(n_, "robot_type", "uav");
	joy_in = getParam<std::string>(n_, "joy_in", "/joy");
	cmd_out = getParam<std::string>(n_, "cmd_out", "/cmd_vel");
	init_height = getParam<double>(n_, "init_height", 0);
	max_height = getParam<double>(n_, "max_height", 10);
	scale_step = getParam<double>(n_, "scale_step", 0.1);
	lin_vel_scale = getParam<double>(n_, "lin_vel_scale", 0.5);
	joy_sub = n.subscribe(joy_in, 50, &JoyToTwist::joy_cb, this);
	cmd_pub = n.advertise<geometry_msgs::Twist>(cmd_out,50);
	current_height = init_height;
}

JoyToTwist::~JoyToTwist()
{
	// Nothing to do?
}

void JoyToTwist::joy_cb(const sensor_msgs::JoyConstPtr& msg)
{
	ROS_INFO("Received Joy msgs");

	if(robot_type.compare("uav") == 0)
	{
		geometry_msgs::Twist cmd;
		cmd.linear.x = lin_vel_scale*msg->axes[1];
		cmd.linear.y = 0;

		cmd.angular.x = 0;
		cmd.angular.y = 0;
		cmd.angular.z = msg->axes[0];
		double actual_height = current_height; 
		current_height =  current_height + scale_step * msg->axes[3];
		if(current_height > max_height)
		{
			cmd.linear.z = actual_height;
			current_height = actual_height;

		}
		else
		{
			cmd.linear.z = current_height;
		}
		
		cmd_pub.publish(cmd);
	}
	else
	{	
		ROS_INFO("Robot type unknown");
	}

	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, ros::this_node::getName());

	JoyToTwist joy_to_twist;

	ros::spin();

	return 0;
}
