#include <iostream>
#include <cstdlib>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

typedef pcl::PointXYZ PointType;

void usage(const std::string &call) {
    std::cout << call << " FILE_NAME" << std::endl;
    exit(1);
}

void filter(pcl::PointCloud<PointType>::ConstPtr const &input_cloud, pcl::PointCloud<PointType>::Ptr &filtered_cloud) {
    pcl::VoxelGrid<PointType> downsample;
    downsample.setInputCloud(input_cloud);
    downsample.setLeafSize(01.0, 01.0, 01.0); //TODO 0.1 cm
    downsample.filter(*filtered_cloud);
}

void plane_filter(pcl::PointCloud<PointType>::ConstPtr const &input_cloud, pcl::PointCloud<PointType>::Ptr &filtered_cloud) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<PointType> segmentator;
    segmentator.setOptimizeCoefficients(true); //TODO optional
    segmentator.setModelType(pcl::SACMODEL_PLANE);
    segmentator.setMethodType(pcl::SAC_RANSAC);
    segmentator.setDistanceThreshold(10.0); //TODO 1.0 cm hardcoded

    segmentator.setInputCloud(input_cloud);
    segmentator.segment(*inliers, *coefficients);

    pcl::ExtractIndices<PointType> cloud_extractor(new pcl::ExtractIndices<PointType>);
    cloud_extractor.setInputCloud(input_cloud);
    cloud_extractor.setIndices(inliers);
    cloud_extractor.setNegative(true);

    cloud_extractor.filter(*filtered_cloud);
}

void clusterExtraction(pcl::PointCloud<PointType>::ConstPtr const &input_cloud, std::vector<pcl::PointCloud<PointType>::Ptr> &clusters) {

    /* Extract cluster indices */

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(input_cloud);

    pcl::EuclideanClusterExtraction<PointType> clusterer;
    clusterer.setClusterTolerance(15.0); //TODO 1.5 cm, hardcoded
    clusterer.setMinClusterSize(100); //TODO hardcoded
    clusterer.setMaxClusterSize(25000); //TODO hardcoded
    clusterer.setSearchMethod(tree);
    clusterer.setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    clusterer.extract(cluster_indices);

    std::cout << cluster_indices.size() << " clusters detected" << std::endl;

    /* Extract point clouds using cluster indices */

    pcl::ExtractIndices<PointType> cloud_extractor(new pcl::ExtractIndices<PointType>);
    cloud_extractor.setInputCloud(input_cloud);
    for (std::vector<pcl::PointIndices>::const_iterator cluster_it = cluster_indices.begin(); cluster_it != cluster_indices.end(); ++cluster_it) {
        pcl::PointIndicesPtr indices = boost::make_shared<pcl::PointIndices> (*cluster_it); //TODO, copy, it's not efficient, check nop_destructor alternative hack

        cloud_extractor.setIndices(indices);
        cloud_extractor.setNegative(false);

        pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>);

        cloud_extractor.filter(*cluster);
        clusters.push_back(cluster);
    }
}

int main (int argc, char *argv[]) {
    std::string file_name;
    pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);

    // Parse command line arguments
    if (argc < 2) {
        usage(argv[0]);
    }
    file_name = argv[1];

    // Load cloud from file
    std::cout << "Loading file" << std::endl;
    if (pcl::io::loadPCDFile(file_name, *input_cloud) < 0) {
        std::string error = "Couldn't read file '" + file_name + "'.\n";
        PCL_ERROR(error.c_str());
        return -1;
    }

    pcl::VoxelGrid<PointType> downsample;
    pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
    downsample.setInputCloud(input_cloud);
    downsample.setLeafSize(01.0, 01.0, 01.0); //TODO 0.1 cm
    downsample.filter(*downsampled_cloud);

    std::vector<pcl::PointCloud<PointType>::Ptr> clusters;
    plane_filter(input_cloud, filtered_cloud);
    clusterExtraction(filtered_cloud, clusters);

    for (size_t i = 0; i < clusters.size(); ++i) {
        std::cerr << "Saving cluster " << i << " to file...";
        std::stringstream convert;
        convert << i;
        pcl::io::savePCDFile("cloud_" + convert.str() + ".pcd", *clusters[i], false);
        std::cerr << "done!" << std::endl;
    }
    return 0;
}
