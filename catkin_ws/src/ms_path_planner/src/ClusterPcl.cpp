#include <ms_path_planner/ClusterPcl.h>
#include <ms_path_planner/NormalEstimationPcl.h>

//Used as criteria to sort a vector
struct myclass {
  bool operator() (int i,int j) { return (i<j);}
} myobject;



template<typename PointT>
double ClusterPcl<PointT>::normal_clustering_thres = 0.1;

template<typename PointT>
ClusterPcl<PointT>::ClusterPcl()
{
}

template<typename PointT>
ClusterPcl<PointT>::~ClusterPcl()
{
}


bool enforceNormalSimilarity (const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
  if (fabs (point_a_normal.dot (point_b_normal)) <0.1)
    return (true);
  return (false);
}

template<typename PointT>
bool ClusterPcl<PointT>::in_filteredList(int p){
	for (int i=0; i<pointIdxFilterOut.size(); i++){
		if (i==p)
			return true;
		if (i>p)
			return false;
	}
	return false;
}

template<typename PointT>
void ClusterPcl<PointT>::cluster_pcl()
{
	clusters_info.clear();
	for(int i=0; i<pcl_in.size(); i++){

		float angle = acos(pcl_in[i].normal[2]);
			//cout<<"angle: "<<angle<<endl;
			uint8_t r,g,b;
			while(angle<0) angle+=2*M_PI;

			if (angle <=0.2 && pcl_in[i].normal[2] >= 0){
				//cout<<"plane"<<endl;
				pcl_noWall.push_back(pcl_in[i]);
				clusters_info.push_back(1);

			}
			else if(angle > M_PI/4 && angle <= 2*M_PI/4){
				pcl_Wall.push_back(pcl_in[i]);

			}
			else if((angle > 0.2 && angle <=M_PI/4) ){
				pcl_noWall.push_back(pcl_in[i]);
				clusters_info.push_back(4);

			}
			else {
				pcl_Wall.push_back(pcl_in[i]);
			}

	}
}

template<typename PointT>
void ClusterPcl<PointT>::filter_Wall(){
	pointIdxFilterOut.clear();
	pcl::VoxelGrid<PointOut> sor;
	PointCloud new_pcl_wall;
	cout<<"pcl_Wall size before downsampling: "<<pcl_Wall.size()<<endl;
	sor.setInputCloud (pcl_Wall.makeShared());
	sor.setLeafSize (0.05f, 0.05f, 0.05f);
	sor.filter (new_pcl_wall);
	pcl::copyPointCloud(new_pcl_wall, pcl_Wall);
	cout<<"pcl_Wall size after downsampling: "<<pcl_Wall.size()<<endl;

	KdTreeIn kdtree_Wall;
	kdtree_Wall.setInputCloud(pcl_Wall.makeShared());
	double radius = 0.2;
	for (int i=0; i<pcl_Wall.size(); i++){
		PointOut p = pcl_Wall[i];
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		int n = kdtree_Wall.radiusSearch(p,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance);
		if (n<18){
			pointIdxFilterOut.push_back(i);
		}
		else{
			double z_min = pcl_Wall[pointIdxRadiusSearch[0]].z;
			double z_max = pcl_Wall[pointIdxRadiusSearch[0]].z;

			for (int j=0; j<pointIdxRadiusSearch.size(); j++){
				if (pcl_Wall[pointIdxRadiusSearch[j]].z < z_min)
					z_min = pcl_Wall[pointIdxRadiusSearch[j]].z;
				if (pcl_Wall[pointIdxRadiusSearch[j]].z > z_max)
					z_max = pcl_Wall[pointIdxRadiusSearch[j]].z;
			}
			if (z_max - z_min < 0.1){
				pointIdxFilterOut.push_back(i);
			}

		}
	}

	ROS_INFO("pointIdxFilterOut size: %d",pointIdxFilterOut.size());

	if (pointIdxFilterOut.size() > 0){
		for (int i=pointIdxFilterOut.size(); i>0; i--){
			std::swap(pcl_Wall[pointIdxFilterOut[i-1]],pcl_Wall.back());
			pcl_Wall.points.pop_back();
		}
	}
	pcl_Wall.width = pcl_Wall.size();
	ROS_INFO("pcl Wall size after filtering: %d",pcl_Wall.size());

}



template<typename PointT>
void ClusterPcl<PointT>::elaborate_cluster()
{
	for(int i=0; i<pcl_noWall.size(); i++){

		float angle = acos(pcl_noWall[i].normal[2]);
		//cout<<"angle: "<<angle<<endl;
		int classification = 0;
		uint8_t r,g,b;
		while(angle<0) angle+=2*M_PI;

		if (angle <=0.2 && pcl_noWall[i].normal[2] >= 0){
			//cout<<"plane"<<endl;
			r = 0;
			g = 255;
			b = 0;
			pcl_noWall.points[i].r = r;
			pcl_noWall.points[i].b = b;
			pcl_noWall.points[i].g = g;


		}
		else if(angle > M_PI/3 && angle <= 3*M_PI/4){

			//cout<<"wall"<<endl;
			r = 255;
			g = 0;
			b = 0;
			pcl_noWall.points[i].r = r;
			pcl_noWall.points[i].b = b;
			pcl_noWall.points[i].g = g;


		}
		else if((angle > 0.2 && angle <=M_PI/3) ){
			//cout<<"ramp-stair"<<endl;
			r = 0;
			g = 127;
			b = 255;
			pcl_noWall.points[i].r = r;
			pcl_noWall.points[i].b = b;
			pcl_noWall.points[i].g = g;

		}
		else {
			r = 255;
			g = 165;
			b = 0;
			pcl_noWall.points[i].r = r;
			pcl_noWall.points[i].b = b;
			pcl_noWall.points[i].g = g;

		}
	}

	/*pointIdxFilterOut.clear();
	for (int i = 0; i < clusters.size(); ++i)
	{
		double z_max = pcl_Wall.points[clusters[i].indices[0]].z;
		double z_min = z_max;
		pcl::PointCloud<PointOut> cloud_cluster;
		Eigen::Vector3f normal(0,0,0);
		bool superable = true;

		for (int j = 0; j< clusters[i].indices.size(); ++j){
			PointOut p = pcl_Wall.points[clusters[i].indices[j]];

			if (p.z > z_max)
				z_max = p.z;
			if (p.z < z_min)
				z_min = p.z;
		}

		if (z_max - z_min <= 0.3){
			for (int j = 0; j< clusters[i].indices.size(); ++j){
				pointIdxFilterOut.push_back(i);
			}
		}
	}

	cout<<"pointIdxFilterOut size:"<<pointIdxFilterOut.size()<<endl;
	cout<<"pcl_Wall before filter size:"<<pcl_Wall.size()<<endl;

	std::sort (pointIdxFilterOut.begin(), pointIdxFilterOut.end(), myobject);

	for (int i= pointIdxFilterOut.size(); i>0; i--){
			PointOut p = pcl_Wall.points[pointIdxFilterOut[i-1]];
			std::swap(pcl_Wall.points[pointIdxFilterOut[i-1]], pcl_Wall.points.back());
			pcl_Wall.points.pop_back();
	}
	cout<<"pcl_Wall after filter size:"<<pcl_Wall.size()<<endl;*/

}





template<typename PointT>
void ClusterPcl<PointT>::set_input_pcl(const pcl::PointCloud<PointOut>& input_pcl)
{
	pcl_in = input_pcl;
	cout<<"input_pcl size: "<<input_pcl.size()<<endl;
	pcl_noWall.clear();
	pcl_Wall.clear();

	pcl_noWall.header.frame_id = pcl_in.header.frame_id;
	pcl_Wall.header.frame_id = pcl_in.header.frame_id;

	pcl_noWall.header.stamp = pcl_in.header.stamp;
	pcl_Wall.header.stamp = pcl_in.header.stamp;


}

template<typename PointT>
void ClusterPcl<PointT>::set_kdtree(const KdTreeIn& input_kdtree)
{
	kdtree_input_pcl = input_kdtree;
}

template<typename PointT>
int ClusterPcl<PointT>::clustering(PointCloud& noWall, PointCloud& borders, PointCloud& segmented, PointCloud& wall)
{
	if (pcl_in.size() > 0)
	{
		noWall.clear();
		borders.clear();
		cout<<"Start clustering"<<endl;
		cluster_pcl();
		cout<<"Finish clustering"<<endl;
		filter_Wall();
		filter_Wall();
		elaborate_cluster();
		noWall = pcl_noWall;
		segmented = pcl_in;
		wall = pcl_Wall;
		return 0;
	}
	else
		return -1;
}

template<typename PointT>
void ClusterPcl<PointT>::get_cluster_info(std::vector<int>& clusters_info_)
{
	clusters_info_ = clusters_info;
}

template class ClusterPcl<pcl::PointXYZRGBNormal>;
template class ClusterPcl<pcl::PointXYZ>;
template class ClusterPcl<pcl::PointXYZI>;

