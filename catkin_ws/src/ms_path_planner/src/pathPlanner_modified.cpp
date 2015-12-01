#include <ros/ros.h>

#include <ms_path_planner/PathPlanning.h>
#include <ms_path_planner/MarkerController.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/thread.hpp>



bool wall_flag = false;
ros::NodeHandle nh("~");


pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> wall_kdTree;
pcl::PointCloud<pcl::PointXYZRGBNormal> wall_pcl;

//-------------------------------------------------------------------------------------------------
//ho dichiarato queste variabili globali per poter chiamare planner->set_input(..) da dove mi pare
//-------------------------------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZI> traversability_pcl;
pcl::KdTreeFLANN<pcl::PointXYZI> traversability_kdtree;
size_t robot_idx;
//-------------------------------------------------------------------------------------------------

PathPlanning* planner;
ros::Publisher path_pub;
tf::TransformListener *tf_;

bool goal_selection = false;

bool getRobotPose(tf::StampedTransform& robot_pose){

    if (tf_->waitForTransform("/map", "base_link", ros::Time(),
                              ros::Duration(1.0))) {
        try {
            tf_->lookupTransform("/map", "base_link", ros::Time(),
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

    //pcl::PointCloud<pcl::PointXYZI> traversability_pcl;
    pcl::fromROSMsg(traversability_msg, traversability_pcl);
    ROS_INFO("path planner input set");
    if (traversability_pcl.size() > 0 && wall_flag){

        tf::StampedTransform robot_pose;
        getRobotPose(robot_pose);
        pcl::PointXYZI robot;
        robot.x = robot_pose.getOrigin().x();
        robot.y = robot_pose.getOrigin().y();
        robot.z = robot_pose.getOrigin().z();

        //pcl::KdTreeFLANN<pcl::PointXYZI> traversability_kdtree;
        traversability_kdtree.setInputCloud(traversability_pcl.makeShared());

        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        traversability_kdtree.nearestKSearch(robot, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        robot_idx = pointIdxNKNSearch[0];


        //----------------------------------------------------------------------------------------------------------------
        //ho commentato questa parte perchè vogliamo disaccoppiare il planning dall'acquisizione della Point Cloud
        //----------------------------------------------------------------------------------------------------------------
        //		planner->set_input(traversability_pcl, wall_pcl, wall_kdTree, traversability_kdtree, pointIdxNKNSearch[0]);
        //		wall_flag = false;
        //		if (goal_selection){

        //			nav_msgs::Path path;

        //			ROS_INFO("compute path");
        //			if(goal_selection){
        //				if(planner->planning(path)){
        //					path_pub.publish(path);

        //				}
        //				else{
        //					ROS_INFO("no path exist for desired goal, please choose another goal");
        //					goal_selection = true;
        //				}
        //				ROS_INFO("path_computed");
        //			}
        //		}
    }
}

void wallCallback(const sensor_msgs::PointCloud2& wall_msg){
    pcl::fromROSMsg(wall_msg, wall_pcl);
    wall_kdTree.setInputCloud(wall_pcl.makeShared());
    wall_flag = true;
}

//-------------------------------------------------------------------------------
//
//void goalSelectionCallback(geometry_msgs::PoseStamped goal_){
//	pcl::PointXYZI  p;
//	p.x = goal_.pose.position.x;
//	p.y = goal_.pose.position.y;
//	p.z = goal_.pose.position.z;
//	planner->set_goal(p);
//	goal_selection = true;
//	ROS_INFO("goal selection");
//}
//
//-------------------------------------------------------------------------------


//-------------------------------------------------------------------------------
//questa funzione converte un geometry_msgs::Pose in un pcl::PointXYZI
//-------------------------------------------------------------------------------
pcl::PointXYZI convert(geometry_msgs::Pose& vector){

    pcl::PointXYZI out;

    out.x = vector.position.x;
    out.y = vector.position.y;
    out.z = vector.position.z;

    return out;

}
//--------------------------------------------------------------------------------


//--------------------------------------------------------------------------------
//questa callback riceve in ingresso l'array di waypoints e per ogni coppia di
//waypoints adiacenti invoca il planner
//--------------------------------------------------------------------------------
void goalSelectionCallback(geometry_msgs::PoseArray waypoints){

    //dimensione dell'array di waypoints
    size_t n = waypoints.poses.size();

    for( size_t i = 0; i < n; i++){

        //istanzio un planner per ogni coppia di waypoints
        PathPlanning new_planner(nh);
        planner = &new_planner;
        nav_msgs::Path path;

        //al primo step il punto iniziale è la posa del robot
        if( i == 0 ) {
            planner->set_input(traversability_pcl,wall_pcl,wall_kdTree,traversability_kdtree,robot_idx);
        }
        //agli step successivi il punto iniziale è il goal dello step precedente
        else {
            pcl::PointXYZI in_point = convert(waypoints.poses.at(i-1));

            //faccio il KNearestNeighbor search giusto per utilizzare la planner->set_input(..) scritta dagli altri
            //poi dobbiamo scriverci la nostra set_input(..) e tutto questo non sarà più necessario
            pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
            kdtree.setInputCloud(traversability_pcl.makeShared());

            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);
            kdtree.nearestKSearch(in_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
            size_t input_idx = pointIdxNKNSearch[0];
            planner->set_input(traversability_pcl,wall_pcl,wall_kdTree,traversability_kdtree,input_idx);
        }

        //qui setto il goal
        pcl::PointXYZI goal_point = convert(waypoints.poses.at(i));
        planner->set_goal(goal_point);
        goal_selection = true;
        ROS_INFO("goal selection");

        //qui lancio il planner
        if(planner->planning(path)){//<--questa funzione va riscritta!!!
            path_pub.publish(path);
        }
        else{
            ROS_INFO("no path exist for desired goal, please choose another goal");
            goal_selection = true;
        }
        ROS_INFO("path_computed");
    }

}
//----------------------------------------------------------------------------------------


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_cluster");


    tf::TransformListener tf;
    tf_ = &tf;

    tf::TransformListener tf_listener(ros::Duration(10.0));

    ros::Subscriber sub_trav = nh.subscribe("/trav/traversability", 1, pointCloudCallback);
    ros::Subscriber sub_wall = nh.subscribe("/clustered_pcl/wall", 1, wallCallback);
    //ros::Subscriber sub_goal = n.subscribe("/goal_topic", 1, goalSelectionCallback);
    ros::Subscriber sub_goal = nh.subscribe<geometry_msgs::PoseArray>("/goal_topic", 1, goalSelectionCallback);

    path_pub = nh.advertise<nav_msgs::Path>("/robot_path",1);

    MarkerController marker;
    ros::spin();

    marker.reset();
    return 0;
}
