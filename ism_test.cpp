#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/recognition/implicit_shape_model.h>
#include <pcl/recognition/impl/implicit_shape_model.hpp>

#include "filter/downsampler.h"
#include "filter/plane_filter.h"
#include "filter/pass_through_filter.h"
#include "cloud_clusterer/euclidean_clusterer.h"

int main (int argc, char* argv[]) {

    /* Parse command line arguments */

    std::string model_file_name;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    unsigned int testing_class;
    bool verbose = false;

    if (argc == 4) {
        model_file_name = std::string(argv[1]);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloud) == -1)
            return -1;
        testing_class = (unsigned int)(strtol(argv[3], NULL, 10));
    }
    else if (argc == 5) {
        if (!std::string(argv[1]).compare("-v")) verbose = true;
        model_file_name = std::string(argv[2]);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[3], *cloud) == -1)
            return -1;
        testing_class = (unsigned int)(strtol(argv[4], NULL, 10));
    }
    else
        return -1;

    /* Load model */

    pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal>::ISMModelPtr model(new pcl::features::ISMModel);

    model->loadModelFromfile(model_file_name);

    /* Apply filters */

    std::vector<Filter<pcl::PointXYZ>*> filters;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    filters.push_back(new Downsampler<pcl::PointXYZ>(01.0, 01.0, 01.0)); // size_x: 0.1 cm, size_y: 0.1 cm, size_z: 0.1 cm,           TODO hardcoded
    filters.push_back(new PassThroughFilter<pcl::PointXYZ>("z", 0.0, 1200.0)); // field_name: "z", min: -100.0 cm, max: 100.0 cm, TODO hardcoded
    filters.push_back(new PlaneFilter<pcl::PointXYZ>(true, 10.0)); // optimize_coefficients: true, threshold: 1.0 cm,                 TODO hardcoded

    for (size_t i = 0; i < filters.size(); ++i) {
        filters[i]->filter(cloud, filtered_cloud);
        cloud.swap(filtered_cloud);
    }

    /* Estimate normals */

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setRadiusSearch(25.0);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    normal_estimator.setInputCloud(cloud);
    normal_estimator.compute(*normals);

    /* Initialize feature estimator method */

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >::Ptr fpfh(new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >());
    fpfh->setRadiusSearch(30.0);
    pcl::Feature< pcl::PointXYZ, pcl::Histogram<153> >::Ptr feature_estimator(fpfh);

    /* Find class objects in the cloud using given model */

    pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal> ism;
    ism.setFeatureEstimator(feature_estimator);
    ism.setSamplingSize(2.0f);

    boost::shared_ptr<pcl::features::ISMVoteList<pcl::PointXYZ> > vote_list = ism.findObjects(model, cloud, normals, testing_class);

    /* Find strongest peaks */

    double radius = model->sigmas_[testing_class]*10.0;
    double sigma = model->sigmas_[testing_class];
    std::vector<pcl::ISMPeak, Eigen::aligned_allocator<pcl::ISMPeak> > strongest_peaks;
    vote_list->findStrongestPeaks(strongest_peaks, testing_class, radius, sigma);

    /* Visalization */

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;
    pcl::PointXYZRGB point;

    if (verbose) {
        colored_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

        colored_cloud->height = 0;
        colored_cloud->width = 1;

        point.r = 100;
        point.g = 100;
        point.b = 100;

        /* Copy the point cloud (requried as they have different point types */

        for (size_t i_point = 0; i_point < cloud->points.size(); i_point++) {
            point.x = cloud->points[i_point].x;
            point.y = cloud->points[i_point].y;
            point.z = cloud->points[i_point].z;
            colored_cloud->points.push_back (point);
        }
        colored_cloud->height += cloud->points.size ();
    }
    else {
        colored_cloud = vote_list->getColoredCloud(cloud);
    }

	point.r = 0;
	point.g = 255;
	point.b = 0;

    point.x = strongest_peaks[0].x;
    point.y = strongest_peaks[0].y;
    point.z = strongest_peaks[0].z;
    colored_cloud->points.push_back(point);

    std::cout << "Peak: [" << point.x << ", " << point.y << ", " << point.z << "] - density: " << strongest_peaks[0].density << std::endl;

	pcl::visualization::CloudViewer viewer("Result viewer");

	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped()) {
	}

	return 0;
}
