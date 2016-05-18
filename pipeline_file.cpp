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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

#include "filter/downsampler.h"
#include "filter/plane_filter.h"
#include "filter/pass_through_filter.h"
#include "cloud_clusterer/euclidean_clusterer.h"

typedef pcl::PointXYZ PointType;

double cloud_colors[3][3] = {   0, 127, 127,
                              127, 127,   0,
                              127,   0, 127 }; //TODO hardcoded

void usage(const std::string &call) {
    std::cout << call << " FILE_NAME" << std::endl;
    exit(1);
}

int main (int argc, char *argv[]) {

    /* Parse command line arguments */

    std::string file_name;

    if (argc < 2) {
        usage(argv[0]);
    }
    file_name = argv[1];

    /* Initialize visualization */

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    viewer->addCube(-625, 625, -625, 625, 0, 1200, 255, 255, 255, "render_area", 0); //TODO hardcoded
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "render_area", 0);

    /* Load cloud from file */

    pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);

    std::cout << "Loading file '" << file_name << "'..." << std::endl;
    if (pcl::io::loadPCDFile(file_name, *input_cloud) < 0) {
        std::string error = "Couldn't read file '" + file_name + "'.\n";
        PCL_ERROR(error.c_str());
        exit(-1);
    }

    /* Apply filters */

    std::vector<Filter<PointType>*> filters;
    pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);

    filters.push_back(new Downsampler<PointType>(01.0, 01.0, 01.0)); // size_x: 0.1 cm, size_y: 0.1 cm, size_z: 0.1 cm,           TODO hardcoded
    filters.push_back(new PassThroughFilter<PointType>("z", 0.0, 1200.0)); // field_name: "z", min: -100.0 cm, max: 100.0 cm, TODO hardcoded
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

    /* Visualization */

    for (size_t i = 0; i < clusters.size(); ++i) {
        std::stringstream convert;
        convert << i;

        pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(clusters[i], cloud_colors[i%3][0], cloud_colors[i%3][1], cloud_colors[i%3][2]);
        viewer->addPointCloud<PointType>(clusters[i], color_handler, convert.str());
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100, false);
    }

    for (size_t i = 0; i < clusters.size(); ++i) {
        std::cerr << "Saving cluster " << i << " to file...";
        std::stringstream convert;
        convert << i;
        pcl::io::savePCDFile("cloud_" + convert.str() + ".pcd", *clusters[i], false);
        std::cerr << "done!" << std::endl;
    }
    viewer->close();

    return 0;
}
