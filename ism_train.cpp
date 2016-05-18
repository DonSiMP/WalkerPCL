#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/recognition/implicit_shape_model.h>
#include <pcl/recognition/impl/implicit_shape_model.hpp>

int main (int argc, char* argv[]) {
    if (argc == 0 || argc % 2 == 0)
        return -1;

    unsigned int n_train_clouds = argc/2;

    /* Load clouds, compute normals and prepare training vectors */

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setRadiusSearch(25.0);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> training_clouds;
    std::vector<pcl::PointCloud<pcl::Normal>::Ptr> training_normals;
    std::vector<unsigned int> training_classes;

    for (unsigned int i = 0; i < n_train_clouds; i++) {
        std::cout << "Loading cloud " << i << " -- Training class " << argv[i*2 + 2] << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile <pcl::PointXYZ> (argv[i*2 + 1], *cloud) == -1)
            return -1;

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        normal_estimator.setInputCloud(cloud);
        normal_estimator.compute (*normals);

        unsigned int training_class = (unsigned int)(strtol(argv[i*2 + 2], NULL, 10));

        training_clouds.push_back (cloud);
        training_normals.push_back (normals);
        training_classes.push_back (training_class);
    }

    std::cout << "Done" << std::endl;

    /* Initialize feature estimator method */

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >::Ptr fpfh(new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >());
    fpfh->setRadiusSearch(30.0);
    pcl::Feature< pcl::PointXYZ, pcl::Histogram<153> >::Ptr feature_estimator(fpfh);

    /* Train the model */

    std::cout << "Training the model" << std::endl;

    pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal> ism;
    ism.setFeatureEstimator(feature_estimator);
    ism.setTrainingClouds(training_clouds);
    ism.setTrainingNormals(training_normals);
    ism.setTrainingClasses(training_classes);
    ism.setSamplingSize(2.0f);

    pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal>::ISMModelPtr model(new pcl::features::ISMModel);
    ism.trainISM(model);

    std::cout << "Done" << std::endl;

    std::string file ("trained_ism_model.txt");
    model->saveModelToFile(file);

    return 0;
}
