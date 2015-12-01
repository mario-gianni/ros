#ifndef NORMALESTIMATIONPCL_H
#define NORMALESTIMATIONPCL_H

#include <ros/ros.h>
#include <ros/console.h>

#include <ms_path_planner/NormalEstimationPclConfig.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <cmath>

using namespace ms_path_planner;

template<typename PointT>
class NormalEstimationPcl
{
private:
	typedef pcl::PointCloud<PointT> PointCloudT;
	typedef pcl::KdTreeFLANN<PointT> KdTreeT;

	NormalEstimationPclConfig config;

	// Kernel between two vectors
	inline float kernel(Eigen::Vector3f& v1, Eigen::Vector3f& v2);
	inline float gaussianKernel(Eigen::Vector3f& v1, Eigen::Vector3f& v2);
	inline float cosineKernel(Eigen::Vector3f& v1, Eigen::Vector3f& v2);

	//Minimun value of the weight in the computation of the weighted covariance matrix
	const float threshold;

	/*Compute the covariance matrix between the point center and his neighbors. The covariance matrix is weighted with
	kernel function*/
	void computeCovarianceMatrix(const PointCloudT& neighbors, const pcl::PointXYZ& center, Eigen::Matrix3f& covariance_matrix);

	//Compute one normal for a given point i
	bool computeNormal(const size_t i, const size_t begin, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, std::vector<bool>& done, const pcl::PointXYZ& center);

	//Compute normals in a given range
	void computeNormalsInRange(const size_t num_thread, const size_t start, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, std::vector<bool>& done, const pcl::PointXYZ& center);

public:
	NormalEstimationPcl();
	~NormalEstimationPcl();
	void computeNormals(PointCloudT& pcl, KdTreeT& kdtree, const pcl::PointXYZ& center);

	inline void setConfig(const NormalEstimationPclConfig& new_config) {config = new_config;}
	inline NormalEstimationPclConfig getConfig() {return config;}
};

#endif //NORMALESTIMATIONPCL_H
