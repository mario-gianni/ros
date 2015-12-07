/*
 * StigmergyPlanner.cpp
 *
 *  Created on: Jun 27, 2013
 *      Author: alcor
 */
#include <stdexcept>
#include <stigmergy_planner/StigmergyPlanner.h>
#include <ms_trajectory_control_msgs/TrajectoryControlGoal.h>
#include <pcl/kdtree/kdtree_flann.h>

StigmergyPlanner::StigmergyPlanner():starting_pose_ready(false),drawn_path(false),finished(false)
{
	path_pub = node.advertise<nav_msgs::Path>("/stigmergy_path",1);
	final_path_pub = node.advertise<nav_msgs::Path>("/final_stigmergy_path",1);
	path_ovelaid_pub = node.advertise<nav_msgs::Path>("/overlaid_path",1);
	path_interp_pub = node.advertise<nav_msgs::Path>("/interpolated_path",1);
	target_pub = node.advertise<visualization_msgs::Marker>("/target",1);
	pcl_sub = node.subscribe("/point_map",1,&StigmergyPlanner::pcl_callback,this);
	server.reset( new interactive_markers::InteractiveMarkerServer("stigmergy_planner","",false) );
	menu_handler.insert("Select Starting Pose",boost::bind(&StigmergyPlanner::processFeedback,this,_1));
	menu_handler.insert("Undo Starting Pose",boost::bind(&StigmergyPlanner::processFeedback,this,_1));
	menu_handler.insert("Apply Plan",boost::bind(&StigmergyPlanner::processFeedback,this,_1));
	menu_handler.insert("Discard Plan",boost::bind(&StigmergyPlanner::processFeedback,this,_1));
	menu_handler.insert("Undo Planning",boost::bind(&StigmergyPlanner::processFeedback,this,_1));
	menu_handler.insert("Evaluate Plan",boost::bind(&StigmergyPlanner::processFeedback,this,_1));

	ac = new TrajectoryControlClient("trajectory_control_action_server",true);

	while(!getRobotPose(robot_pose))
	{
		ROS_INFO("Waiting for transformation");
	}

	makeViewFacingMarker(robot_pose);
	server->applyChanges();

	target.header.frame_id = "/map";
	target.header.stamp = ros::Time::now();

	target.id = 1;
	target.ns = "target";

	target.scale.x = 0.3;
	target.scale.y = 0.3;
	target.scale.z = 0.3;
	target.type = visualization_msgs::Marker::MESH_RESOURCE;
	target.mesh_resource = "package://stigmergy_planner/meshes/barrier.dae";
	target.mesh_use_embedded_materials = true;
	target.lifetime = ros::Duration(0);

}

StigmergyPlanner::~StigmergyPlanner()
{}

void StigmergyPlanner::reset()
{
	server.reset();
}

void StigmergyPlanner::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& map)
{
	pcl::fromROSMsg(*map, point_cloud_in);
	point_cloud_in_ptr = point_cloud_in.makeShared();
	ROS_INFO("PCL Received");
	ROS_INFO("PCL Size [%lu]",point_cloud_in_ptr->points.size());
}

void StigmergyPlanner::overlay(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl, nav_msgs::Path in, nav_msgs::Path &out)
{
	ROS_INFO("overlay PCL Size [%lu]",point_cloud_in_ptr->points.size());
	if(pcl->points.size() != 0)
	{
		out.header.stamp = in.header.stamp;
		out.header.frame_id = in.header.frame_id;
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
		kdtree->setInputCloud(pcl);
		int num_points = in.poses.size();
		int i = 0;
		int K = 1;
		double threshold = 0.1;
		while(i < num_points)
		{
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			pcl::PointXYZ searchPoint;
			searchPoint.x = in.poses[i].pose.position.x;
			searchPoint.y = in.poses[i].pose.position.y;
			searchPoint.z = in.poses[i].pose.position.z;
			kdtree->nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquaredDistance);
			geometry_msgs::PoseStamped point;
			point.header.stamp = in.poses[i].header.stamp;
			point.header.frame_id = in.poses[i].header.frame_id;
			if(pointNKNSquaredDistance[0] <= threshold)
			{
				point.pose.position.x = pcl->points[pointIdxNKNSearch[0]].x;
				point.pose.position.y = pcl->points[pointIdxNKNSearch[0]].y;
				point.pose.position.z = pcl->points[pointIdxNKNSearch[0]].z;
			}
			else
			{
				point.pose.position.x = searchPoint.x;
				point.pose.position.y = searchPoint.y;
				point.pose.position.z = searchPoint.z;
			}

			point.pose.orientation.x = in.poses[i].pose.orientation.x;
			point.pose.orientation.y = in.poses[i].pose.orientation.y;
			point.pose.orientation.z = in.poses[i].pose.orientation.z;
			point.pose.orientation.w = in.poses[i].pose.orientation.w;
			out.poses.push_back(point);
			i++;
		}
	}
	else
	{
		out = in;
	}
}

void StigmergyPlanner::interpolate(nav_msgs::Path in, nav_msgs::Path &out)
{
	out.header.frame_id = in.header.frame_id;
	out.header.stamp = in.header.stamp;

	int num_points = in.poses.size();

	out.poses.push_back(in.poses[0]);
	out.poses.push_back(in.poses[1]);

	double c[5] = {-1./16.,4./16.,10./16.,4./16.,-1./16.};

	for(int i = 2; i < num_points - 2; i++)
	{
		geometry_msgs::PoseStamped p;
		p.header.stamp = in.poses[i].header.stamp;
		p.header.frame_id = in.poses[i].header.frame_id;
		for(int j = 0; j < 5; j++)
		{
			p.pose.position.x += c[j]*in.poses[i+j-2].pose.position.x;
			p.pose.position.y += c[j]*in.poses[i+j-2].pose.position.y;
			p.pose.position.z += c[j]*in.poses[i+j-2].pose.position.z;
		}
		p.pose.orientation.x = 0;
		p.pose.orientation.y = 0;
		p.pose.orientation.z = 0;
		p.pose.orientation.w = 1;
		out.poses.push_back(p);

	}

	out.poses.push_back(in.poses[num_points - 2]);
	out.poses.push_back(in.poses[num_points - 1]);

}

// %Tag(processFeedback)%
void StigmergyPlanner::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      if(!finished && !starting_pose_ready && feedback->menu_entry_id == 1)
      {
    	  starting_pose.header.frame_id = "/map";
    	  starting_pose.header.stamp = ros::Time::now();
    	  starting_pose.pose = feedback->pose;
    	  starting_pose_ready = true;
    	  ROS_INFO("Starting Pose Selected");

    	  path3D.header.frame_id = "/map";
    	  path3D.header.stamp = ros::Time::now();
    	  path3D.poses.clear();
    	  path_pub.publish(path3D);
      }
      if(starting_pose_ready && feedback->menu_entry_id == 2)
      {
    	  starting_pose_ready = false;
    	  drawn_path = false;
    	  finished = false;
    	  path3D.poses.clear();
    	  path_pub.publish(path3D);
    	  ROS_INFO("Starting Pose Canceled");
      }
      if(drawn_path && !finished && feedback->menu_entry_id == 3)
      {
    	  finished = true;
    	  target.pose = path3D.poses[path3D.poses.size() - 1].pose;
    	  ROS_INFO("feedback PCL Size [%lu]",point_cloud_in_ptr->points.size());
    	  overlay(point_cloud_in_ptr,path3D,path_overlaid);
    	  interpolate(path_overlaid,path_interp);
    	  target.action = visualization_msgs::Marker::ADD;
    	  target_pub.publish(target);
    	  path_pub.publish(path3D);
    	  final_path_pub.publish(path3D);
    	  path_ovelaid_pub.publish(path_overlaid);
    	  path_interp_pub.publish(path_interp);
    	  ROS_INFO("Plan Completed");

    	  ac->waitForServer();
    	  ms_trajectory_control_msgs::TrajectoryControlGoal goal;
    	  goal.path = path3D;
    	  ac->sendGoal(goal);
    	  /*ac->waitForResult();
    	  if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    	  {
    		  ROS_INFO("Plan Completed");
    	  }
    	  else
    	  {
    		  ROS_INFO("Plan not completed");
    	  }*/
      }

      if(finished && feedback->menu_entry_id == 4)
      {
    	  starting_pose_ready = false;
    	  drawn_path = false;
    	  finished = false;
    	  path3D.poses.clear();
    	  path_overlaid.poses.clear();
    	  path_interp.poses.clear();
    	  target.action = visualization_msgs::Marker::DELETE;
    	  target_pub.publish(target);
    	  path_pub.publish(path3D);

    	  ac->waitForServer(ros::Duration(5.0));
    	  ac->cancelGoal();
    	  ROS_INFO("Plan Canceled");
      }

      if(drawn_path && !finished && feedback->menu_entry_id == 5)
      {
    	  if(path3D.poses.size() < 50)
    	  {
    		  path3D.poses.clear();
    		  geometry_msgs::Pose init;
    		  init.position.x = 0.0;
    		  init.position.y = 0.0;
    		  init.position.z = 0.0;
    		  init.orientation.x = 0;
    		  init.orientation.y = 0;
    		  init.orientation.z = 0;
    		  init.orientation.w = 1;
    		  server->setPose(int_marker.name,init);
    	  }
    	  else
    	  {
    		  double new_dim = path3D.poses.size() - 50;
    		  path3D.poses.resize(new_dim);
    		  server->setPose(int_marker.name,path3D.poses[new_dim - 1].pose);

    	  }
    	  path_pub.publish(path3D);
    	  ROS_INFO("Undo");
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      if(starting_pose_ready && !finished)
      {
    	  geometry_msgs::PoseStamped pose;
    	  pose.header.frame_id = feedback->header.frame_id;
    	  pose.header.stamp = feedback->header.stamp;
    	  pose.pose = feedback->pose;
    	  path3D.poses.push_back(pose);
    	  path_pub.publish(path3D);
    	  drawn_path = true;
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}

bool StigmergyPlanner::getRobotPose(tf::StampedTransform& robot_pose) {

	std::string robot_frame_id("/base_link");
	std::string global_frame_id("/map");

	if(tf_.waitForTransform(global_frame_id,robot_frame_id,ros::Time(),ros::Duration(1.0)))
	{
		try
		{
			tf_.lookupTransform(global_frame_id,robot_frame_id,ros::Time(),robot_pose);
		}
		catch(tf::LookupException& ex) {
			ROS_INFO("No Transform available Error looking up robot pose: %s\n", ex.what());
			return false;
		}
		catch(tf::ConnectivityException& ex) {
			ROS_INFO("Connectivity Error looking up robot pose: %s\n", ex.what());
			return false;
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_INFO("Extrapolation Error looking up robot pose: %s\n", ex.what());
			return false;
		}

		return true;
	}
	else
	{
		ROS_INFO("Transformation is not available");
		return false;
	}
}


Marker StigmergyPlanner::makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://stigmergy_planner/meshes/hikingstick3.dae";
  marker.mesh_use_embedded_materials = true;
  marker.scale.x = msg.scale * 0.3;
  marker.scale.y = msg.scale * 0.3;
  marker.scale.z = msg.scale * 0.3;

  /*marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0*/;

  return marker;
}

InteractiveMarkerControl& StigmergyPlanner::makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void StigmergyPlanner::makeViewFacingMarker(tf::StampedTransform robot_pose)
{
  int_marker.header.frame_id = robot_pose.frame_id_;
  int_marker.header.stamp = robot_pose.stamp_;

  int_marker.pose.position.x = robot_pose.getOrigin().getX();
  int_marker.pose.position.y = robot_pose.getOrigin().getY();
  int_marker.pose.position.z = robot_pose.getOrigin().getZ();
  int_marker.scale = 1;

  int_marker.name = "view_facing";
  int_marker.description = "3D Planning pencil";

  InteractiveMarkerControl control;

  // make a control that rotates around the view axis
  //control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  //control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  //control.orientation.w = 1;
  //control.name = "rotate";

  //int_marker.controls.push_back(control);

  // create a box in the center which should not be view facing,
  // but move in the camera plane.
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;
  control.name = "move";

  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;

  int_marker.controls.push_back(control);

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = "menu";
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, boost::bind(&StigmergyPlanner::processFeedback,this,_1));
  menu_handler.apply(*server, int_marker.name);
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"stygmergy_planner");

	StigmergyPlanner planner;

	ros::spin();

	planner.reset();

	return 0;
}


