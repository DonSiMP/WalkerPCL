#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <dirent.h>

#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointType;

bool first = true;
bool next = false;

void usage(const std::string &call) {
    std::cout << call << " DIR_NAME" << std::endl;
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

    std::string directory_name;

    if (argc == 2) {
        directory_name = std::string(argv[1]);
    }
    else
        usage(argv[0]);

    /* Initialize visualization */

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    viewer->registerKeyboardCallback(keyboardCallback);
    viewer->addCube(-625, 625, -625, 625, 0, 1200, 255, 255, 255, "render_area", 0); //TODO hardcoded
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "render_area", 0);

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

        /* Visualization */

        pcl::visualization::PointCloudColorHandlerRGBField<PointType> color_handler(input_cloud);
        if (first) {
            viewer->addPointCloud<PointType>(input_cloud, color_handler, "0");
            first = false;
        }
        else
            viewer->updatePointCloud<PointType>(input_cloud, color_handler, "0");

        while (!next && !viewer->wasStopped()) {
            viewer->spinOnce(100, false);
        }
        if (viewer->wasStopped()) {
            std::cout << "Exiting application..." << std::endl;
            exit(0);
        }
        next = false;

    }
    viewer->close();

    return 0;
}
