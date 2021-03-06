#ifndef CLUSTERPCL_H
#define CLUSTERPCL_H

#include <vector>

#include <ms_path_planner/ClusterPclConfig.h>

using namespace ms_path_planner;

#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>

using namespace std;





template<typename PointT>
class ClusterPcl{

public:
	typedef pcl::PointXYZRGBNormal PointOut;
	typedef pcl::PointCloud<PointOut> PointCloud;
	typedef pcl::KdTreeFLANN<PointOut> KdTreeIn;

	ClusterPcl();
	~ClusterPcl();

	void set_input_pcl(const PointCloud& input_pcl);
	void set_kdtree(const KdTreeIn& input_kdtree);
	int clustering(PointCloud& noWall, PointCloud& borders, PointCloud& segmented, PointCloud& wall);
	void get_cluster_info(std::vector<int>& cluster_info_);

	inline void setConfig(ClusterPclConfig& new_config) {config = new_config; normal_clustering_thres = new_config.normal_clustering_thres;}
	inline ClusterPclConfig getConfig() {return config;}


private:
	ClusterPclConfig config;

	// this param needs to be static
	static double normal_clustering_thres;

	//Input point cloud
	PointCloud pcl_in;

	//Point cloud with normals
	PointCloud pcl_normal;

	//Kdtree of the input point cloud
	KdTreeIn kdtree_input_pcl;

	//Point cloud without wall
	PointCloud pcl_noWall, pcl_Wall;

	//Index of point filtered out in noWall
	std::vector<int> pointIdxFilterOut;

	//cluster is an array of indices-arrays, indexing points of the input point cloud and own cluster
	pcl::IndicesClusters clusters;


	std::vector<int> clusters_info;




	bool in_filteredList(int p);

	void compute_normals();

	void filter_Wall();

	void elaborate_cluster();

	void cluster_pcl();

	bool enforceSimilarity (const PointOut& point_a, const PointOut& point_b, float squared_distance);

};

#endif
