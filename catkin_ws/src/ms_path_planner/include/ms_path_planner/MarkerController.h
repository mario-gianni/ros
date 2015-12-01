#ifndef MARKER_CONTROLLER_H_
#define MARKER_CONTROLLER_H_

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

using namespace visualization_msgs;
class MarkerController{
public:
	std::string marker_name;
	InteractiveMarker int_marker;
	ros::Publisher goal_pub;
	MarkerController();
	MarkerController(std::string);
	MarkerController(std::string,std::string);
	MarkerController(std::string,std::string,std::string);
	~MarkerController();

	boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
	interactive_markers::MenuHandler menu_handler;

	Marker makeBox(InteractiveMarker&);
	InteractiveMarkerControl& makeBoxControl(InteractiveMarker&);
	void makeViewFacingMarker();
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
	void reset();
};


#endif //MARKER_CONTROLLER
