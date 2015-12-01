#include <ms_path_planner/TravAnal.h>

TravAnal::TravAnal()
{
}

TravAnal::~TravAnal()
{
}

inline double point_plane_distance(pcl::PointXYZRGBNormal& p, pcl::ModelCoefficients& coefficients)
{
	double num = abs(p.x*coefficients.values[0] + p.y*coefficients.values[1] + p.z*coefficients.values[2] + coefficients.values[3]);
	double den = sqrt(pow(coefficients.values[0],2) + pow(coefficients.values[1],2)  +pow(coefficients.values[2],2));
	return num/den;
}

inline double point_point_distance(pcl::PointXYZRGBNormal p1, pcl::PointXYZRGBNormal p2)
{
	return sqrt(pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2) + pow(p1.z - p2.z,2));
}

void TravAnal::set_input(std::vector<int>& clusters_info_, pcl::PointCloud<pcl::PointXYZRGBNormal>& wall_pcl_, pcl::PointCloud<pcl::PointXYZRGBNormal>& input_pcl_)
{
	noWall_pcl = input_pcl_;
	wall_pcl= wall_pcl_;
	clusters_info = clusters_info_;
	wall_kdtree.setInputCloud(wall_pcl.makeShared());
	input_kdtree.setInputCloud(noWall_pcl.makeShared());
}

void TravAnal::computeTrav(PointCloudI& traversabilty_pcl_)
{
	traversabilty_pcl.clear();
	traversabilty_pcl.header.frame_id = "/map";

	roughness_pcl.clear();
	roughness_pcl.header.frame_id = "/map";

	label_pcl.clear();
	label_pcl.header.frame_id = "/map";

	clearence_pcl.clear();
	clearence_pcl.header.frame_id = "/map";

	density_pcl.clear();
	density_pcl.header.frame_id = "/map";

	for (int i=0; i<noWall_pcl.size(); i++){
		if(clusters_info[i]!= -1){
			double wL = computeLabel(clusters_info[i]);

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			pcl::PointXYZRGBNormal p = noWall_pcl[i];
			//input_kdtree.radiusSearch(p,0.2,pointIdxRadiusSearch,pointRadiusSquaredDistance);
			input_kdtree.radiusSearch(p,config.density_radius_multiplier*config.leaf_size,pointIdxRadiusSearch,pointRadiusSquaredDistance);

			pcl::PointCloud<pcl::PointXYZRGBNormal> nh;
			for (int k=0; k<pointIdxRadiusSearch.size(); k++){
				nh.push_back(noWall_pcl[pointIdxRadiusSearch[k]]);
			}
			double wR = computeRough(i, nh);
			double wD = computeDensity(i, nh);
			double wC = computeClearance(i);

			//cout<<"cost wR: "<<wR<<" cost wD: "<<wD<<" cost wc: "<<wC<<endl;

			double wTot = wL*(wD  + wR + wC)/3;
			//cout<<"trav cost: "<<wTot<<endl;

			if (wTot < 2){

				PointOutI point;
				point.x = noWall_pcl[i].x;
				point.y = noWall_pcl[i].y;
				point.z = noWall_pcl[i].z;

				point.intensity = wTot;
				traversabilty_pcl.push_back(point);

				point.intensity = wD;
				density_pcl.push_back(point);

				point.intensity = wR;
				roughness_pcl.push_back(point);

				point.intensity = wL;
				label_pcl.push_back(point);

				point.intensity = wC;
				clearence_pcl.push_back(point);

			}

		}


	}
	traversabilty_pcl_ = traversabilty_pcl;
}

double TravAnal::computeRough(int point_index, pcl::PointCloud<pcl::PointXYZRGBNormal>& nh)
{
	if(nh.size()>3){
		double tot = 0;
		pcl::ModelCoefficients coefficients;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;

		seg.setOptimizeCoefficients (true);

		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.01);

		seg.setInputCloud (nh.makeShared());
		seg.segment (*inliers, coefficients);

		for (int i=0; i< nh.size(); i++){
			double d = point_plane_distance(nh[i], coefficients);
			tot +=  d;
		}

		return min(10*tot/(nh.size()),1.0);
	}
	else
		return 1;
}

inline double TravAnal::computeDensity(int point_index, pcl::PointCloud<pcl::PointXYZRGBNormal>& nh)
{
	//double density = nh.size()*pow(config.leaf_size,3)/(4*M_PI*pow(3 * config.leaf_size,3)/3);

//	double expected = M_PI*pow(config.densityRadius,2)/pow(config.leaf_size,2);
//	double density = max(min(1.0,2*(nh.size()/expected - 0.5)),0.0);

	double density = nh.size()*pow(config.leaf_size,3)/(4*M_PI*pow(config.density_radius_multiplier*config.leaf_size,3)/3);
	return 2/(125*density);
}

double TravAnal::computeClearance(int point_index)
{
	pcl::PointXYZRGBNormal p;
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);
	double threshold = 0.1;
	double inf = INFINITY;
	wall_kdtree.nearestKSearch(noWall_pcl[point_index],1,pointIdxNKNSearch,pointNKNSquaredDistance);
	double dist = pointNKNSquaredDistance[0];

	return (dist < threshold) ? 3 : (threshold/dist);

}
double TravAnal::computeLabel(int classification){
	switch (classification)
	{
	case 1:
		return 0.7;

	case 2:
		return 0.8;

	case 3:
	    return 1;
	default:
	    return 1;
	}
	return 1;
}

void TravAnal::get_pcl(PointCloudI& clearence_pcl_, PointCloudI& density_pcl_, PointCloudI& label_pcl_, PointCloudI& roughness_pcl_){
	clearence_pcl_ = clearence_pcl;
	density_pcl_ = density_pcl;
	label_pcl_ = label_pcl;
	roughness_pcl_ = roughness_pcl;
}

