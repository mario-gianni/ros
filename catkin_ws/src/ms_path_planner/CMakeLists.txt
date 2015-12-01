cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")


rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#add dynamic reconfigure api (catkin)
#find_package(catkin REQUIRED dynamic_reconfigure)
#generate_dynamic_reconfigure_options(
#  cfg/DynamicJoinPcl.cfg
#)
#add dynamic reconfigure api (rosbuild)
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_library(normalestimation src/NormalEstimationPcl.cpp)
rosbuild_add_library(dynamicjoinpcl src/DynamicJoinPcl.cpp)
rosbuild_add_library(clusterpcl src/ClusterPcl.cpp)
rosbuild_add_library(conversionpcl src/ConversionPcl.cpp)
rosbuild_add_library(travanalpcl src/TravAnal.cpp)
rosbuild_add_library(pathplanning src/PathPlanning.cpp)
rosbuild_add_library(marker src/MarkerController.cpp)



#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_executable(test_dyn_join_pcl src/test_dyn_join_pcl.cpp)
#rosbuild_add_executable(test_normal src/test_normal.cpp)
#rosbuild_add_executable(test_dyn_join_pcl_w_normal src/test_dyn_join_pcl_w_normal.cpp)
#rosbuild_add_executable(test_cluster src/test_clustering.cpp)
#rosbuild_add_executable(test_traversability src/test_traversability.cpp)
#rosbuild_add_executable(test_spherical_kdtree src/test_spherical_kdtree.cpp)
#rosbuild_add_executable(test_distance_partition src/test_distance_partition.cpp)
#rosbuild_add_executable(test_spherical_partition src/test_spherical_partition.cpp)
#rosbuild_add_executable(test_segmentation src/test_segmentation.cpp)
#rosbuild_add_executable(test_planner src/test_planner.cpp)
rosbuild_add_executable(mapping src/mapping.cpp)
rosbuild_add_executable(traversability src/traversability.cpp)
rosbuild_add_executable(pathPlanner src/pathPlanner.cpp)
rosbuild_add_executable(compute_normals src/compute_normals.cpp)




#rosbuild_add_executable(test_spherical_kdtree src/test_spherical_kdtree.cpp)
#rosbuild_add_executable(test_distance_partition src/test_distance_partition.cpp)
#rosbuild_add_executable(test_spherical_partition src/test_spherical_partition.cpp)
#rosbuild_add_executable(test_segmentation src/test_segmentation.cpp)


#target_link_libraries(test_dyn_join_pcl dynamicjoinpcl conversionpcl)
#target_link_libraries(test_normal normalestimation conversionpcl)
#target_link_libraries(test_dyn_join_pcl_w_normal normalestimation conversionpcl dynamicjoinpcl)
#target_link_libraries(test_segmentation normalestimation dynamicjoinpcl)
#target_link_libraries(test_cluster clusterpcl dynamicjoinpcl normalestimation conversionpcl)
#target_link_libraries(test_traversability clusterpcl dynamicjoinpcl normalestimation conversionpcl travanalpcl)
target_link_libraries(mapping dynamicjoinpcl  conversionpcl normalestimation)
target_link_libraries(traversability clusterpcl  conversionpcl travanalpcl)
target_link_libraries(pathPlanner marker  pathplanning)
target_link_libraries(compute_normals conversionpcl normalestimation)


#target_link_libraries(test_planner marker pathplanning clusterpcl dynamicjoinpcl normalestimation conversionpcl travanalpcl)

# make sure configure headers are built before any node using them
#add_dependencies(test_dyn_join_pcl ${PROJECT_NAME}_gencfg)
