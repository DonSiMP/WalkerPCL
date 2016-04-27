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

#include "filter/downsampler.h"
#include "filter/plane_filter.h"
#include "cloud_clusterer/euclidean_clusterer.h"

typedef pcl::PointXYZ PointType;

void usage(const std::string &call) {
    std::cout << call << " FILE_NAME" << std::endl;
    exit(1);
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

    pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>);

    Downsampler<PointType> downsampler(01.0, 01.0, 01.0);
    downsampler.filter(input_cloud, downsampled_cloud);

    pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);

    PlaneFilter<PointType> plane_filter(true, 10.0); // optimize_coefficients: true, threshold: 1.0 cm, TODO hardcoded
    plane_filter.filter(downsampled_cloud, filtered_cloud);

    std::vector<pcl::PointCloud<PointType>::Ptr> clusters;

    EuclideanClusterer<PointType> clusterer(15.0, 100, 25000); // tolerance: 1.5 cm, min: 100, max: 25000, TODO hardcoded
    clusterer.cluster(filtered_cloud, clusters);

    for (size_t i = 0; i < clusters.size(); ++i) {
        std::cerr << "Saving cluster " << i << " to file...";
        std::stringstream convert;
        convert << i;
        pcl::io::savePCDFile("cloud_" + convert.str() + ".pcd", *clusters[i], false);
        std::cerr << "done!" << std::endl;
    }
    return 0;
}
