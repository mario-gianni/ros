#include <ms_path_planner/ConversionPcl.h>
#include <stdio.h>

template<typename PointT>
ConversionPcl<PointT>::ConversionPcl() :
	output_frame("/map"),
	tf_listener(0L)
{
}

template<typename PointT>
ConversionPcl<PointT>::~ConversionPcl()
{
}

template<typename PointT>
void ConversionPcl<PointT>::transform(const sensor_msgs::PointCloud2& msg_in, sensor_msgs::PointCloud2& msg_out)
{
	ros::Time scan_time = msg_in.header.stamp;
	tf::StampedTransform input_output_tf;

	if(!tf_listener)
	{
		ROS_ERROR("ConversionPcl::transform: tf_listener is not set");
		return;
	}

	try
	{
		//tf_listener.lookupTransform(laser_frame, global_frame, scan_time, map_laser_tf);
		tf_listener->lookupTransform(output_frame, msg_in.header.frame_id, scan_time, input_output_tf);
		sensor_msgs::PointCloud2 msg_new_scan;
		pcl_ros::transformPointCloud(output_frame, msg_in, msg_out, *tf_listener);
	}
	catch(tf::LookupException& ex)
	{
		ROS_ERROR("ConversionPcl::transform: tf::LookupException&: %s", ex.what());
		return;
	}
	catch(tf::ExtrapolationException& ex)
	{
		ROS_ERROR("ConversionPcl::transform: tf::ExtrapolationException&: %s", ex.what());
		return;
	}
}

template<typename PointT>
void ConversionPcl<PointT>::transform(const sensor_msgs::PointCloud2& msg_in, PointCloudT& pcl_out)
{
	sensor_msgs::PointCloud2 msg_out;
	transform(msg_in, msg_out);
	pcl::fromROSMsg(msg_out, pcl_out);

	pcl_out.header = pcl_conversions::toPCL(msg_in.header);
	pcl_out.header.frame_id = output_frame;
}

template<typename PointT>
void ConversionPcl<PointT>::transform(const PointCloudT& pcl_in, sensor_msgs::PointCloud2& msg_out)
{
	sensor_msgs::PointCloud2 msg_in;
	pcl::toROSMsg(pcl_in, msg_in);
	transform(msg_in, msg_out);
}

template<typename PointT>
void ConversionPcl<PointT>::transform(const PointCloudT& pcl_in, PointCloudT& pcl_out)
{
	sensor_msgs::PointCloud2 msg_in;
	pcl::toROSMsg(pcl_in, msg_in);
	transform(msg_in, pcl_out);

	pcl_out.header = pcl_conversions::toPCL(msg_in.header);
	pcl_out.header.frame_id = output_frame;
}

template<typename PointT>
void ConversionPcl<PointT>::getLastTransform(tf::Transform& t)
{
	t.setOrigin(input_output_tf.getOrigin());
	t.setRotation(input_output_tf.getRotation());
}

template<typename PointT>
void ConversionPcl<PointT>::getLastTransform(tf::StampedTransform& t)
{
	t = input_output_tf;
}

template<typename PointT>
void ConversionPcl<PointT>::getFrameOrigin(std::string frame_id, pcl::PointXYZ& p)
{
	tf::Stamped<tf::Vector3> origin, point;
	origin.frame_id_ = frame_id;
	origin.setZero();
	point.frame_id_ = output_frame;

	try {
		tf_listener->transformPoint(output_frame, origin, point);
		p.x = point.x();
		p.y = point.y();
		p.z = point.z();
	}
	catch(tf::LookupException& ex)
	{
		ROS_ERROR("ConversionPcl::getFrameOrigin: tf::LookupException&: %s", ex.what());
		return;
	}
	catch(tf::ExtrapolationException& ex)
	{
		ROS_ERROR("ConversionPcl::getFrameOrigin: tf::ExtrapolationException&: %s", ex.what());
		return;
	}

}

template class ConversionPcl<pcl::PointXYZ>;
template class ConversionPcl<pcl::PointXYZRGBNormal>;

