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

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/recognition/implicit_shape_model.h>
#include <pcl/recognition/impl/implicit_shape_model.hpp>

#include "filter/downsampler.h"
#include "filter/plane_filter.h"
#include "filter/pass_through_filter.h"
#include "cloud_clusterer/euclidean_clusterer.h"

typedef pcl::PointXYZ PointType;

bool first = true; //TODO remove global variables
bool next = false;
unsigned int frame = 0;

void usage(const std::string &call) {
    std::cout << call << " FILE_NAME" << std::endl;
    exit(1);
}

void keyboardCallback(const pcl::visualization::KeyboardEvent &event) {
    //std::cout << "Name: " << event.getKeySym() << " Code: " << event.getKeyCode() <<std::endl;
    if (event.keyUp() && event.getKeySym() == "Right") {
        next = true;
    }
}

int main (int argc, char *argv[]) {

    /* Parse command line arguments */

    std::string model_file_name;
    std::string directory_name;

    unsigned int testing_class;
    bool verbose = false;

    if (argc == 4) {
        model_file_name = std::string(argv[1]);
        directory_name = std::string(argv[2]);
        testing_class = (unsigned int)(strtol(argv[3], NULL, 10));
    }
    else if (argc == 5) {
        if (!std::string(argv[1]).compare("-v")) verbose = true;
        model_file_name = std::string(argv[2]);
        directory_name = std::string(argv[3]);
        testing_class = (unsigned int)(strtol(argv[4], NULL, 10));
    }
    else
        usage(argv[0]);

    /* Load model */

    pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal>::ISMModelPtr model(new pcl::features::ISMModel);

    model->loadModelFromfile(model_file_name);

    /* Initialize feature estimator method */

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >::Ptr fpfh(new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >());
    fpfh->setRadiusSearch(30.0);
    pcl::Feature< pcl::PointXYZ, pcl::Histogram<153> >::Ptr feature_estimator(fpfh);

    /* Initialize visualization *

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    viewer->registerKeyboardCallback(keyboardCallback);
    viewer->addCube(-625, 625, -625, 625, 0, 1200, 255, 255, 255, "render_area", 0); //TODO hardcoded
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "render_area", 0);
    */

    /* Iterate over all files in directory */

    std::vector<std::string> file_names;

    DIR *directory = opendir(directory_name.c_str());
    if (directory == NULL) {
        std::cout << "Error(" << errno << ") opening " << directory << std::endl;
        exit(-1);
    }

    std::cout << "Reading directory '" << directory_name << "'..." << std::endl;

    struct dirent *entry;
    while ((entry = readdir(directory)) != NULL) {
        std::string file_name = directory_name + entry->d_name;

        /* Check file extension */

        if (file_name.substr(file_name.find_last_of(".") + 1) != "pcd") {
            std::cout << "Skipping file '" << file_name << "'..." << std::endl;
        }
        else {
            file_names.push_back(file_name);
        }
    }
    std::sort(file_names.begin(), file_names.end());

    for (size_t i = 0; i < file_names.size(); ++i) {
        std::string file_name = file_names[i];

        /* Load cloud from file */

        pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);

        std::cout << "Loading file '" << file_name << "'..." << std::endl;
        if (pcl::io::loadPCDFile(file_name, *input_cloud) < 0) {
            std::string error = "Couldn't read file '" + file_name + "'.\n";
            PCL_ERROR(error.c_str());
            exit(-1);
        }

        /* Filter NaNs */

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, indices);

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

        /* Estimate normals */

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setRadiusSearch(25.0);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        normal_estimator.setInputCloud(filtered_cloud);
        normal_estimator.compute(*normals);

        /* Filter normal NaNs */

        pcl::removeNaNNormalsFromPointCloud(*normals, *normals, indices);
        pcl::copyPointCloud(*filtered_cloud, indices, *filtered_cloud); //TODO better way?

        /* Find class objects in the cloud using given model */

        pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal> ism;
        ism.setFeatureEstimator(feature_estimator);
        ism.setSamplingSize(2.0f);

        boost::shared_ptr<pcl::features::ISMVoteList<pcl::PointXYZ> > vote_list = ism.findObjects(model, filtered_cloud, normals, testing_class);

        /* Find strongest peaks */

        double radius = model->sigmas_[testing_class]*10.0;
        double sigma = model->sigmas_[testing_class];
        std::vector<pcl::ISMPeak, Eigen::aligned_allocator<pcl::ISMPeak> > strongest_peaks;
        vote_list->findStrongestPeaks(strongest_peaks, testing_class, radius, sigma);

        /* Visualization */

            pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;
            pcl::PointXYZRGB point;

            if (!verbose) {
                colored_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

                colored_cloud->height = 0;
                colored_cloud->width = 1;

                point.r = 100;
                point.g = 100;
                point.b = 100;

                /* Copy the point cloud (requried as they have different point types */

                for (size_t i_point = 0; i_point < filtered_cloud->points.size(); i_point++) {
                    point.x = filtered_cloud->points[i_point].x;
                    point.y = filtered_cloud->points[i_point].y;
                    point.z = filtered_cloud->points[i_point].z;
                    colored_cloud->points.push_back (point);
                }
                colored_cloud->height += filtered_cloud->points.size();
            }
            else {
                colored_cloud = vote_list->getColoredCloud(filtered_cloud);
            }

            point.r = 0;
            point.g = 255;
            point.b = 0;

            point.x = strongest_peaks[0].x;
            point.y = strongest_peaks[0].y;
            point.z = strongest_peaks[0].z;
            colored_cloud->points.push_back(point);
            colored_cloud->height++;

        // TODO for all classes

            /*
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler(colored_cloud);
            if (first)
                viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, color_handler, "0");
            else
                viewer->updatePointCloud<pcl::PointXYZRGB>(colored_cloud, color_handler, "0");
        first = false;

        while (!next && !viewer->wasStopped()) {
            viewer->spinOnce(100, false);
        }
        if (viewer->wasStopped()) {
            std::cout << "Exiting application..." << std::endl;
            exit(0);
        }
        next = false;
            */

        std::cerr << "Saving frame " << frame << " to file...";
        std::stringstream convert;
        convert << frame++;
        pcl::io::savePCDFile("cloud_" + convert.str() + ".pcd", *colored_cloud, false);
        std::cerr << "done!" << std::endl;
    }
    /*
    viewer->close();
    */

    return 0;
}
