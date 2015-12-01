#include <ms_path_planner/MarkerController.h>

MarkerController::MarkerController(){
	ros::NodeHandle node;
	goal_pub = node.advertise<geometry_msgs::PoseStamped>("/goal_topic", 1);
	server.reset( new interactive_markers::InteractiveMarkerServer("marker_controller","",false) );
	menu_handler.insert("Select Goal",boost::bind(&MarkerController::processFeedback,this,_1));
	makeViewFacingMarker();
	server->applyChanges();
}

MarkerController::MarkerController(std::string goal_topic_name){
	ros::NodeHandle node;
	goal_pub = node.advertise<geometry_msgs::PoseStamped>(goal_topic_name, 1);
	server.reset( new interactive_markers::InteractiveMarkerServer("marker_controller","",false) );
	menu_handler.insert("Select Goal",boost::bind(&MarkerController::processFeedback,this,_1));
	makeViewFacingMarker();
	server->applyChanges();
}

MarkerController::MarkerController(std::string goal_topic_name, std::string int_server_name){
	ros::NodeHandle node;
	goal_pub = node.advertise<geometry_msgs::PoseStamped>(goal_topic_name, 1);
	server.reset( new interactive_markers::InteractiveMarkerServer(int_server_name,"",false) );
	menu_handler.insert("Select Goal",boost::bind(&MarkerController::processFeedback,this,_1));
	makeViewFacingMarker();
	server->applyChanges();
}

MarkerController::MarkerController(std::string goal_topic_name, std::string int_server_name, std::string m_name){
	marker_name = m_name;
	ros::NodeHandle node;
	goal_pub = node.advertise<geometry_msgs::PoseStamped>(goal_topic_name, 1);
	server.reset( new interactive_markers::InteractiveMarkerServer(int_server_name,"",false) );
	menu_handler.insert("Select Goal",boost::bind(&MarkerController::processFeedback,this,_1));
	makeViewFacingMarker();
	server->applyChanges();
}

MarkerController::~MarkerController(){}

void MarkerController::reset()
{
	server.reset();
}


Marker MarkerController::makeBox(InteractiveMarker& msg){
	Marker marker;
	marker.type = Marker::CUBE;
	marker.scale.x = msg.scale * 0.45;
	marker.scale.y = msg.scale * 0.45;
	marker.scale.z = msg.scale * 0.45;
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	marker.color.a = 1.0;

	return marker;

}

InteractiveMarkerControl& MarkerController::makeBoxControl(InteractiveMarker& msg){
	InteractiveMarkerControl control;
	control.always_visible = true;
	control.markers.push_back( makeBox(msg) );
	msg.controls.push_back( control );

	return msg.controls.back();
}

void MarkerController::makeViewFacingMarker(){
	int_marker.header.frame_id = "/map";
	int_marker.header.stamp = ros::Time::now();

	int_marker.pose.position.x = 0;
	int_marker.pose.position.y = 0;
	int_marker.pose.position.z = 0;
	int_marker.scale = 1;

	int_marker.name = marker_name;
	int_marker.description = "Goal selection " + marker_name;

	InteractiveMarkerControl control;
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
	server->setCallback(int_marker.name, boost::bind(&MarkerController::processFeedback,this,_1));
	menu_handler.apply(*server, int_marker.name);
}
void MarkerController::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback){
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
	      if(feedback->menu_entry_id == 1)
	      {
	    	 geometry_msgs::PoseStamped goal;
	    	 goal.header.frame_id = "/map";
	    	 goal.header.stamp = feedback->header.stamp;
	    	 goal.pose = feedback->pose;
	    	 goal_pub.publish(goal);

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
