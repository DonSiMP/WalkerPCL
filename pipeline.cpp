#include <iostream>
#include <cstdlib>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <dirent.h>

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
#include "filter/pass_through_filter.h"
#include "cloud_clusterer/euclidean_clusterer.h"

typedef pcl::PointXYZ PointType;

void usage(const std::string &call) {
    std::cout << call << " FILE_NAME" << std::endl;
    exit(1);
}

int main (int argc, char *argv[]) {

    /* Parse command line arguments */

    std::string directory_name;

    if (argc < 2) {
        usage(argv[0]);
    }
    directory_name = argv[1];

    /* Iterate over all files in directory */

    DIR *directory = opendir(directory_name.c_str());
    if (directory == NULL) {
        std::cout << "Error(" << errno << ") opening " << directory << std::endl;
        //TODO exit
    }

    std::cout << "Reading directory '" << directory_name << "'..." << std::endl;

    struct dirent *entry;
    while ((entry = readdir(directory)) != NULL) {

        std::string file_name = directory_name + entry->d_name;

        /* Check file extension */

        if (file_name.substr(file_name.find_last_of(".") + 1) != "pcd") {
            std::cout << "Skipping file '" << file_name << "'..." << std::endl;
            continue;
        }

        /* Load cloud from file */

        pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);

        std::cout << "Loading file '" << file_name << "'..." << std::endl;
        if (pcl::io::loadPCDFile(file_name, *input_cloud) < 0) {
            std::string error = "Couldn't read file '" + file_name + "'.\n";
            PCL_ERROR(error.c_str());
            return -1;
        }

        /* Apply filters */

        std::vector<Filter<PointType>*> filters;
        pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);

        filters.push_back(new Downsampler<PointType>(01.0, 01.0, 01.0)); // size_x: 0.1 cm, size_y: 0.1 cm, size_z: 0.1 cm,           TODO hardcoded
        filters.push_back(new PassThroughFilter<PointType>("z", -1000.0, 1000.0)); // field_name: "z", min: -100.0 cm, max: 100.0 cm, TODO hardcoded
        filters.push_back(new PlaneFilter<PointType>(true, 10.0)); // optimize_coefficients: true, threshold: 1.0 cm,                 TODO hardcoded

        for (size_t i = 0; i < filters.size(); ++i) {
            filters[i]->filter(input_cloud, filtered_cloud);
            input_cloud.swap(filtered_cloud);
        }
        input_cloud.swap(filtered_cloud);

        /* Extract clusters */

        std::vector<pcl::PointCloud<PointType>::Ptr> clusters;

        EuclideanClusterer<PointType> clusterer(15.0, 100, 25000); // tolerance: 1.5 cm, min: 100, max: 25000,                        TODO hardcoded
        clusterer.cluster(filtered_cloud, clusters);

        /* Debug */

        for (size_t i = 0; i < clusters.size(); ++i) {
            std::cerr << "Saving cluster " << i << " to file...";
            std::stringstream convert;
            convert << i;
            pcl::io::savePCDFile("cloud_" + convert.str() + ".pcd", *clusters[i], false);
            std::cerr << "done!" << std::endl;
        }
    }

    return 0;
}
