#ifndef _DOWNSAMPLER_H_
#define _DOWNSAMPLER_H_

#include <pcl/filters/voxel_grid.h>
#include "filter.h"

/*
 *  Template class that can be used for downsampling a point cloud.
 */
template <class T>
class Downsampler: public Filter<T> {
public:
    // Creator. The object will downsampler the input cloud into voxels of size size_x, size_y, size_z.
    Downsampler(float size_x, float size_y, float size_z);

    // Destructor.
    ~Downsampler();

    // Actual function for downsampling the input cloud.
    void filter(const typename pcl::PointCloud<T>::ConstPtr &input_cloud, typename pcl::PointCloud<T>::Ptr &filtered_cloud);

private:
    pcl::VoxelGrid<T> downsampler;

};

#include "downsampler.tpp"

#endif
