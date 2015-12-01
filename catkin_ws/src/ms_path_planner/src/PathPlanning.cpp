#include <ms_path_planner/PathPlanning.h>

inline double weight(double radius){
	return (0.8/(1+exp((radius - 1)/0.5)) + 0.2);
}

PathPlanning::PathPlanning(ros::NodeHandle n_){
	count = 0;
	n = n_;
	markerArrPub = n.advertise<visualization_msgs::MarkerArray>("PathPlanner/visitedNodes", 1);
	pathPub = n.advertise<nav_msgs::Path>("PathPlanner/localPath", 1);
}

PathPlanning::~PathPlanning(){}


bool myfunction (IdxProbability i,IdxProbability j) { return (i.probability<j.probability); }

inline double PathPlanning::dist(Point& p1, Point& p2){
	return sqrt(pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2) + pow(p1.z - p2.z,2));
}

//Create the neighborood for the currentPoint and the probability associate with each neighbor
void PathPlanning::find_neighbors(std::vector<IdxProbability>& neighbors, double& radius){

	// Find the nearest point labeling with wall and set the search radius according to this distance
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);
	Point point_noWall = noWall[nodes[currentNode].pointIdx];
	pcl::PointXYZRGBNormal point_wall;
	point_wall.x = point_noWall.x;
	point_wall.y = point_noWall.y;
	point_wall.z = point_noWall.z;

	wallKdTree.nearestKSearch(point_wall, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	radius = pointNKNSquaredDistance[0];
	////ROS_INFO("RADIUS: %f", radius);
	// Neighbors within radius search
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	noWallKdTree.radiusSearch(noWall[nodes[currentNode].pointIdx], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	/***
	 *  Extract the neighbors and compute probability associates with each point.
	 *  Point with low cost will have higher probability	  */
	float normal = 0;
	for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {

		if (dist(noWall[pointIdxRadiusSearch[i]], goal) < 0.001) {
			neighbors.clear();
			IdxProbability point;
			point.pointIdx = pointIdxRadiusSearch[i];
			point.probability = 1;
			neighbors.push_back(point);
			normal = 1;
			break;
		}
		else if (noWall[pointIdxRadiusSearch[i]].intensity < INFINITY) {
			float prob = 1/noWall[pointIdxRadiusSearch[i]].intensity;
			normal += prob;
			IdxProbability point;
			point.pointIdx = pointIdxRadiusSearch[i];
			point.probability = prob;
			neighbors.push_back(point);
		}

	}
	////ROS_INFO("neighbors size: %d", neighbors.size());

	//Normalize the probability
	for(int i=0; i<neighbors.size(); i++){
		neighbors[i].probability /= normal;
	}

	//Sort the nighbors by probability
	std::sort (neighbors.begin(), neighbors.end(), myfunction);

	//Compute comulative probability
	for(int i=1; i<neighbors.size()-1; i++){
		neighbors[i].probability += neighbors[i-1].probability;
	}
	neighbors.back().probability = 1;

}

bool PathPlanning::visited_point(size_t pointIdx) {
	for (std::vector<size_t>::iterator it = visitedNodes.begin() ; it != visitedNodes.end(); ++it){
		if(nodes[*it].pointIdx == pointIdx)
			return true;
	}
	return false;
}

//Sample the followers from the current point p
void PathPlanning::sampling_followers(){
	//Find neighbors
	std::vector<IdxProbability> neighbors;
	double radius;
	find_neighbors(neighbors, radius);

	//Sampling the child from the neighbors if the neighbors are more then a th, else use all the neighbors

		int count = 0, test = 0;
		double w = weight(radius);
		while (count < neighbors.size()*w && test < neighbors.size()){
			float r = r = ((double) rand() / (RAND_MAX));
			int i=0;
			while(neighbors[i].probability<r){
				i++;
			}
			test++;
			if (!visited_point(neighbors[i].pointIdx)){
				PointPlanning child;
				double heuristic = dist(noWall.points[neighbors[i].pointIdx],goal);
				child.id = nodes.size();
				child.cost = dist(noWall[nodes[currentNode].pointIdx], noWall[neighbors[i].pointIdx])
						+ noWall[neighbors[i].pointIdx].intensity + heuristic;
				child.parent = currentNode;
				child.pointIdx = neighbors[i].pointIdx;
				nodes[currentNode].child.push_back(child.id);
				nodes.push_back(child);
				leafNodes.push_back(child.id);
				visitedNodes.push_back(child.id);
				visualization_msgs::Marker marker;
				marker.header.frame_id = "/map";
				marker.header.stamp  =  ros::Time::now();
				marker.type = marker.SPHERE;
				marker.action = visualization_msgs::Marker::ADD;
				marker.scale.x = 0.01;
				marker.scale.y = 0.01;
				marker.scale.z = 0.01;
				marker.color.a = 1.0;
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
				marker.pose.position.x = noWall[neighbors[i].pointIdx].x;
				marker.pose.position.y = noWall[neighbors[i].pointIdx].y;
				marker.pose.position.z = noWall[neighbors[i].pointIdx].z;
				marker.id = markerArr.markers.size() + 1;
				markerArr.markers.push_back(marker);
				count++;
				std::cout<<count<<std::endl;
			}
		}

		// remove current from leafNodes:
		for (std::vector<size_t>::iterator it = leafNodes.begin() ; it != leafNodes.end(); ++it){
			if(*it == currentNode)
			{
				std::swap(*it, leafNodes.back());
				leafNodes.pop_back();
                break;
			}
		}

		////ROS_INFO("visited node size: %d", markerArr.markers.size());
		markerArrPub.publish(markerArr);
		//Add the current point to the path
}

//Find the best node from all the leaf node
bool PathPlanning::find_next_expansion(){
	double minCost = INFINITY;
	size_t minCostNodeIdx;
	bool found = false;
	//Find the best expansaion within the leaf nodes.
	for (std::vector<size_t>::iterator it = leafNodes.begin() ; it != leafNodes.end(); ++it){
		if (nodes[*it].cost < minCost)
		{
			minCost = nodes[*it].cost;
			minCostNodeIdx = *it;
			found = true;
		}
	}
	if (found){
		currentNode = minCostNodeIdx;
	}
	return found;
}

inline bool PathPlanning::check_goal(){
	return (dist(noWall[nodes[currentNode].pointIdx], goal) < 0.1);
}

void PathPlanning::set_input(PointCloud& noWall_, pcl::PointCloud<pcl::PointXYZRGBNormal>& wall_,
		pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>& wallKdTree_, pcl::KdTreeFLANN<Point>& noWallKdTree_, int robotIdx_){
	noWall = noWall_;
	std::cout<<"noWall size: "<<noWall.size()<<std::endl;
	wall = wall_;
	wallKdTree = wallKdTree_;
	noWallKdTree = noWallKdTree_;
	robotIdx = robotIdx_;
	nodes.clear();
	PointPlanning node;
	node.id = nodes.size();
	node.cost = 0;
	node.parent = node.id;
	node.pointIdx = robotIdx;
	currentNode = node.id;
	nodes.push_back(node);
	visitedNodes.push_back(node.id);
}

void PathPlanning::set_goal(Point& goal_){
	// Find the goal nearest point in noWall point cloud
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);
	visitedNodes.clear();
	leafNodes.clear();
        nodes.erase(nodes.begin()+1, nodes.end());
	noWallKdTree.nearestKSearch(goal_, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	goal = noWall[pointIdxNKNSearch[0]];
	for (int i=0; i<markerArr.markers.size(); i++ ){
		markerArr.markers[i].action = visualization_msgs::Marker::DELETE;
	}
}

bool PathPlanning::planning(nav_msgs::Path& path_){
	path_.poses.clear();
	path_.header.frame_id = "/map";

	visitedNodes.clear();
	leafNodes.clear();
    	nodes.erase(nodes.begin()+1, nodes.end());
	nav_msgs::Path path2_;
	markerArr.markers.clear();

	bool findPath = false;
	bool existPath = true;
	while(!findPath && existPath){
		count++;
		sampling_followers();
		existPath = find_next_expansion();
		findPath = check_goal();
	}
	if(findPath) {
		// build path:
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = "/map";
		pose.pose.position.x = goal.x;
		pose.pose.position.y = goal.y;
		pose.pose.position.z = goal.z;
		path2_.poses.push_back(pose);
		size_t node = currentNode;
		do {
			Point point = noWall[nodes[node].pointIdx];
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = "/map";
			pose.pose.position.x = point.x;
			pose.pose.position.y = point.y;
			pose.pose.position.z = point.z;
			path2_.poses.push_back(pose);
			node = nodes[node].parent;
		} while(node);
		
		for (int i=path2_.poses.size(); i>0; i--){
			path_.poses.push_back(path2_.poses[i-1]);
		}
	}
	else {
		ROS_WARN("NO PATH");
	}
	
	//ROS_INFO("path length: %d",path_.poses.size());
	pathPub.publish(path_);
	return existPath;
}


