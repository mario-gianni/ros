#include <ms_path_planner/NormalEstimationPcl.h>
#include <ms_path_planner/ZTimer.h>
#include <pcl/filters/extract_indices.h>


#define NORM3_L2(x, y, z) (sqrt(pow((x), 2) + pow((y), 2) + pow((z), 2)))

#ifdef LOG_TIMES
#define LOG_TIME(obj, name, ...) (obj).measureAndLog(name, __VA_ARGS__)
#else
#define LOG_TIME(obj, name, ...)
#endif

template<typename PointT>
NormalEstimationPcl<PointT>::NormalEstimationPcl() :
		threshold(0.5)
{
}

template<typename PointT>
NormalEstimationPcl<PointT>::~NormalEstimationPcl()
{
}

template<typename PointT>
inline float NormalEstimationPcl<PointT>::kernel(Eigen::Vector3f& v1, Eigen::Vector3f& v2)
{
	switch(config.kernel_type)
	{
	case NormalEstimationPcl_Gaussian: return gaussianKernel(v1, v2);
	case NormalEstimationPcl_Cosine:   return cosineKernel(v1, v2);
	default:			               return 0;
	}
}

template<typename PointT>
inline float NormalEstimationPcl<PointT>::gaussianKernel(Eigen::Vector3f& v1, Eigen::Vector3f& v2)
{
	//Implement the standard gaussian kernel: k(v1,v2) = exp(-||v1-v2||^2 / h)
	return exp(-pow((v1 - v2).norm(), 2) / config.smoothing);
}

template<typename PointT>
inline float NormalEstimationPcl<PointT>::cosineKernel(Eigen::Vector3f& v1, Eigen::Vector3f& v2)
{
	return pow((1+cos(M_PI*(v1 - v2).norm()/config.radius))*0.5, config.smoothing);
}

template<typename PointT>
void NormalEstimationPcl<PointT>::computeCovarianceMatrix(const PointCloudT& neighbors, const pcl::PointXYZ& center, Eigen::Matrix3f& covariance_matrix)
{
	Eigen::Vector3f c(center.x, center.y, center.z);
	covariance_matrix.setZero();
	for(int i = 0; i < neighbors.size(); i++) {
		Eigen::Vector3f v(neighbors[i].x, neighbors[i].y, neighbors[i].z);
		Eigen::Vector3f diff = v - c;
		covariance_matrix += kernel(v, c) * (diff * diff.transpose());
	}
	covariance_matrix /= neighbors.size();
}

template<typename PointT>
bool NormalEstimationPcl<PointT>::computeNormal(const size_t i, const size_t begin, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, std::vector<bool>& done, const pcl::PointXYZ& center)
{
	if(done[i]) return true;


	Eigen::Vector3f pseudonormal(Eigen::Vector3f(center.x, center.y, center.z) - Eigen::Vector3f(pcl[i].x, pcl[i].y, pcl[i].z));
	pseudonormal.normalize();

#ifndef USE_PSEUDONORMALS_AS_NORMALS
	//Find nighbors of point
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	if(kdtree.radiusSearch(pcl[i], config.radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 5) {

#if 0
		/* if there are too many neighbors, try to downsample at random: */
		size_t old_size = pointIdxRadiusSearch.size(), num_removed = 0;
		while(pointIdxRadiusSearch.size() > 10)
		{
			size_t index = rand() % pointIdxRadiusSearch.size();
			//std::swap(*(pointIdxRadiusSearch.begin() + index), pointIdxRadiusSearch.back());
			//pointIdxRadiusSearch.pop_back();
			//std::swap(*(pointRadiusSquaredDistance.begin() + index), pointRadiusSquaredDistance.back());
			//pointRadiusSquaredDistance.pop_back();
			pointIdxRadiusSearch.erase(pointIdxRadiusSearch.begin() + index);
			pointRadiusSquaredDistance.erase(pointRadiusSquaredDistance.begin() + index);
			num_removed++;
		}
		if(num_removed)
		{
			ROS_INFO("Random downsampling removed %d points (initial neighbor count was %d)", num_removed, old_size);
		}
#endif


		ZTimer ztimer;

		//If neighbors are more than 5 compute the covariance_matrix
		//cout<<"neghbors size:"<<pointIdxRadiusSearch.size()<<endl;

		PointCloudT neighbors;
		pcl::PointXYZ baricenter(0.0, 0.0, 0.0);
		size_t size = 0;
		for(size_t j = 0; j < pointIdxRadiusSearch.size(); j++) {
			PointT point_j = pcl[pointIdxRadiusSearch[j]];
			neighbors.push_back(point_j);
			baricenter.x += point_j.x;
			baricenter.y += point_j.y;
			baricenter.z += point_j.z;
			size++;
		}
		baricenter.x /= size;
		baricenter.y /= size;
		baricenter.z /= size;

		LOG_TIME(ztimer, "build_neighbours", size);

		typedef Eigen::Matrix3f MatrixT;

		MatrixT covariance_matrix;
		computeCovarianceMatrix(neighbors, baricenter, covariance_matrix);

		LOG_TIME(ztimer, "covariance_matrix", size);

//
//		//From the covariance matrix compute the normal using PCA with Eigeinvalue decomposition
//		Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix, true);
//
//
//
//		float e0 = es.eigenvalues()(0).real();
//		float e1 = es.eigenvalues()(1).real();
//		float e2 = es.eigenvalues()(2).real();
//
//
//
//		float min_e = std::min(e0,e1);
//		min_e = std::min(min_e,e2);
//		Vector3cf v1c,v2c;
//		if (min_e==e0){
//			v1c = es.eigenvectors().col(1);
//			v2c = es.eigenvectors().col(2);
//		}
//		else if(min_e==e1){
//			v1c = es.eigenvectors().col(0);
//			v2c = es.eigenvectors().col(2);
//		}
//		else{
//			v1c = es.eigenvectors().col(0);
//			v2c = es.eigenvectors().col(1);
//		}

		//Eigen::JacobiSVD<MatrixT> svd(covariance_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::JacobiSVD<MatrixT> svd(covariance_matrix, Eigen::ComputeFullV);
		Eigen::Vector3f Sv = svd.singularValues();
		MatrixT T = covariance_matrix*svd.matrixV().transpose();
		float e0 = T.col(0).norm();
		float e1 = T.col(1).norm();
		float e2 = T.col(2).norm();
		float min_e = std::min(e0,e1);
		min_e = std::min(min_e,e2);
		Eigen::Vector3f v1c,v2c;
		if(min_e==e0) {
			v1c = T.col(1)/e1;
			v2c = T.col(2)/e2;
		}
		else if(min_e==e1) {

			v1c = T.col(0)/e0;
			v2c = T.col(2)/e2;
		}
		else {
			v1c = T.col(1)/e1;
			v2c = T.col(0)/e0;
		}

//		//Take just the real part
//		std::complex<float> x = vc(0);
//		std::complex<float> y = vc(1);
//		std::complex<float> z = vc(2);
//		Eigen::Vector3f normal(x.real(),y.real(),z.real());

		//Take just the real part of both the eigenvectors
		std::complex<float> x1 = v1c(0);
		std::complex<float> y1 = v1c(1);
		std::complex<float> z1 = v1c(2);
		Eigen::Vector3f v1(x1.real(),y1.real(),z1.real());
		std::complex<float> x2 = v2c(0);
		std::complex<float> y2 = v2c(1);
		std::complex<float> z2 = v2c(2);
		Eigen::Vector3f v2(x2.real(),y2.real(),z2.real());

		//Compute the cross product
		Eigen::Vector3f normal = v1.cross(v2);
		//cout <<"normla "<<i<<": "<<"-"<<normal[0]<<"-"<<normal[1]<<"-"<<normal[2]<<endl;
#if 0
		if(normal.norm() < 1e-3) {
			ROS_INFO("NormalEstimationPcl::computeNormal: point %d: abnormal normal! ", i);
			ROS_INFO("NormalEstimationPcl::computeNormal:     e0=%f, e1=%f, e2=%f", e0, e1, e2);
			ROS_INFO("NormalEstimationPcl::computeNormal:     normal: %f, %f, %f", normal(0), normal(1), normal(2));
		}
#endif

		normal.normalize();
		if(normal.dot(pseudonormal) < 0)
		{
			normal = -1 * normal;
		}

		pcl[i].normal[0] = normal(0);
		pcl[i].normal[1] = normal(1);
		pcl[i].normal[2] = normal(2);
		done[i] = true;

		LOG_TIME(ztimer, "svd", size);

		if (3 * Sv[2] / (Sv[0] + Sv[1] + Sv[2]) < config.flatness_curvature_threshold) {
			//std::cout<<"find flat aerea"<<std::endl;
			for (int j = 0; j < pointIdxRadiusSearch.size(); j++) {
				pcl::PointXYZ delta;
				delta.x = baricenter.x - pcl[pointIdxRadiusSearch[j]].x;
				delta.y = baricenter.y - pcl[pointIdxRadiusSearch[j]].y;
				delta.z = baricenter.z - pcl[pointIdxRadiusSearch[j]].z;
				if (pointIdxRadiusSearch[j] >= begin
						&& pointIdxRadiusSearch[j] <= end
						&& NORM3_L2(delta.x, delta.y, delta.z) < config.radius*0.5) {
					pcl[pointIdxRadiusSearch[j]].normal[0] = normal(0);
					pcl[pointIdxRadiusSearch[j]].normal[1] = normal(1);
					pcl[pointIdxRadiusSearch[j]].normal[2] = normal(2);
					done[pointIdxRadiusSearch[j]] = true;
				}
			}
		}

		LOG_TIME(ztimer, "block_assign");

		return true;
	}
	else {
		pcl[i].normal[0] = NAN;
		pcl[i].normal[1] = NAN;
		pcl[i].normal[2] = NAN;
		done[i] = true;

		//cout << "NormalEstimationPcl::computeNormal: point " << i << ": isolated point will have no normal " << endl;

		return false;
	}
#else
	pcl[i].normal[0] = pseudonormal(0);
	pcl[i].normal[1] = pseudonormal(1);
	pcl[i].normal[2] = pseudonormal(2);
	done[i] = true;
#endif
}

template<typename PointT>
void NormalEstimationPcl<PointT>::computeNormalsInRange(const size_t num_thread, const size_t start, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, std::vector<bool>& done, const pcl::PointXYZ& center)
{
	ROS_INFO("NormalEstimationPcl::computeNormalsInRange: spawn thread #%d (point range: %d-%d)", num_thread, start, end);

	try
	{
		size_t num_computed_points = 0, num_isolated_points = 0, num_invalid_normals = 0;
		ZTimer ztimer;

		for (size_t i = start; i < end; i++)
		{
			if(!done[i])
			{
				num_computed_points++;
			}

			if(!computeNormal(i, start, end, pcl, kdtree, done, center))
			{
				num_isolated_points++;
			}

			if(NORM3_L2(pcl[i].normal_x, pcl[i].normal_y, pcl[i].normal_z) < 0.99)
			{
				num_invalid_normals++;
			}
		}

		double tt = ztimer.measure();
		ROS_INFO("NormalEstimationPcl::computeNormalsInRange: finished thread #%d: computed %d normals in %fs (%f normal/s), #isolated: %d, #invalid: %d", num_thread, num_computed_points, tt, num_computed_points / tt, num_isolated_points, num_invalid_normals);
	}
	catch(boost::thread_interrupted&)
	{
		ROS_INFO("NormalEstimationPcl::computeNormalsInRange: thread #%d has been interrupted", num_thread);
	}
}

template<typename PointT>
void NormalEstimationPcl<PointT>::computeNormals(PointCloudT& pcl, KdTreeT& kdtree, const pcl::PointXYZ& center)
{
	const size_t n = pcl.size();
	if(n > 0)
	{
		//cout << "NormalEstimationPcl::computeNormals: begin (pcl size: " << n << ")" << endl;
//		for (int i = 0; i < input_pcl.size(); i++) {
//			pcl::PointXYZRGBNormal p;
//			if (compute_normal(input_pcl.points[i], p)){
//				if(!std::isnan(p.normal_x) && !std::isnan(p.normal_y) && !std::isnan(p.normal_z) &&
//						!std::isinf(p.normal_x) && !std::isinf(p.normal_y) && !std::isinf(p.normal_z)){
//					output.push_back(p);
//				}
//			}
//		}

		std::vector<bool> done;

		for (int i = 0; i < pcl.size(); i++) {
			done.push_back(false);
		}
		if(config.num_threads > 1)
		{
			std::vector<boost::shared_ptr<boost::thread> > threads;
			for(size_t num_thread = 0; num_thread < config.num_threads; num_thread++)
			{
				size_t begin = n * num_thread / config.num_threads,
						 end = n * (num_thread + 1) / config.num_threads;
				threads.push_back(
					boost::make_shared<boost::thread>(
						boost::bind(
							&NormalEstimationPcl::computeNormalsInRange,
							this,
							num_thread,
							begin,
							end,
							boost::ref(pcl),
							boost::ref(kdtree),
							boost::ref(done),
							boost::cref(center)
						)
					)
				);
			}

			for(size_t num_thread = 0; num_thread < threads.size(); num_thread++)
			{
				threads[num_thread]->join();
			}
		}
		else
		{
			computeNormalsInRange(0, 0, n, pcl, kdtree, done, center);
		}
		PointCloudT cloud_out;
		std::vector<int> index;
		pcl::removeNaNNormalsFromPointCloud (pcl, cloud_out, index);
		cloud_out.swap(pcl);

		
		size_t num_done = 0;
		for (size_t i = 0; i < done.size(); i++)
		{
			if(done[i]) num_done++;
		}
		double percent_done = (double)(100*num_done)/(double)done.size();
		ROS_INFO("NormalEstimationPcl::computeNormals: done: %f%%", percent_done);

		size_t num_invalid_normals = 0;
		for (size_t i = 0; i < pcl.size(); i++)
		{
			if(NORM3_L2(pcl[i].normal_x, pcl[i].normal_y, pcl[i].normal_z) < 0.99)
			{
				num_invalid_normals++;
			}
		}

		ROS_INFO("NormalEstimationPcl::computeNormals: finished; # invalid normals: %d", num_invalid_normals);
	}
	else
	{
		ROS_WARN("NormalEstimationPcl::computeNormals: received empty point cloud");
	}
}

template class NormalEstimationPcl<pcl::PointXYZINormal>;
template class NormalEstimationPcl<pcl::PointXYZRGBNormal>;
