#ifndef PATH_PLANNING_H_
#define PATH_PLANNING_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <nav_msgs/Path.h>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>



struct IdxProbability{
	size_t pointIdx;
	float probability;
};

class PathPlanning{

public:
	typedef pcl::PointXYZI Point;
	typedef pcl::PointCloud<Point> PointCloud;

	struct PointPlanning{
		size_t parent;
		size_t id;
		int pointIdx;
		double cost;
		std::vector<size_t> child;
	};


	PathPlanning(ros::NodeHandle n_);
	~PathPlanning();

	//set the input
	void set_input(PointCloud& noWall_, pcl::PointCloud<pcl::PointXYZRGBNormal>& wall_,
	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>& wallKdTree_, pcl::KdTreeFLANN<Point>& noWallKdTree_, int robotIdx_);

	//set the goal position
	void set_goal(Point& goal_);

	//Compute the path if it is exist
	bool planning(nav_msgs::Path& path_);



private:
	//The graph
	std::vector<PointPlanning> nodes;

	//List of the leafe node in the explored graph
	std::vector<size_t>  leafNodes;

	//Visited nodes
	std::vector<size_t>  visitedNodes;

	size_t currentNode;


	//Ros node handle
	ros::NodeHandle n;

	//List of the point composing the path
	std::vector<size_t>  path;

	//Point Clouds of segmented part: wall and noWall, the intensity is the traversability cost of each point
	pcl::PointCloud<pcl::PointXYZRGBNormal> wall;
	PointCloud  noWall;

	//Kd tree representation of the point cloud wall
	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> wallKdTree;
	pcl::KdTreeFLANN<Point>noWallKdTree;

	//Robot pose and goal
	Point goal;

	int robotIdx;

	//Count iteration
	int count;

	//Get the markerArray of visited node
	visualization_msgs::MarkerArray markerArr;

	//Publisher for merkerArr
	ros::Publisher markerArrPub;

	//Publisher for path
	ros::Publisher pathPub;

	//Return the value of the euclidian distane between p1 and p2
	double dist(Point& p1, Point& p2);

	//Find the best node
	bool find_next_expansion();

	//Find nieghbors to current point and comput probability
	void find_neighbors(std::vector<IdxProbability>& neighbors, double& radius);

	bool visited_point(size_t pointIdx);

	//Sample the followers from the current point p
	void sampling_followers();

	//Back track when different expansion is choosen
	void backtracking(int parentIdx);

	bool check_goal();





};



#endif //PATH_PLANNING_H_
