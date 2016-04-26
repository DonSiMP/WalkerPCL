#ifndef _CLOUD_CLUSTERER_H_
#define _CLOUD_CLUSTERER_H_

#include <vector>
#include <pcl/point_cloud.h>

/*
 *  Template generic class that can be used for segmenting a point cloud into clusters.
 */
template <class T>
class CloudClusterer {
public:
    // Destructor.
    virtual ~CloudClusterer() {}

    // Actual method for segmenting the input cloud into clusters.
    virtual void cluster(const typename pcl::PointCloud<T>::ConstPtr &input_cloud, std::vector<typename pcl::PointCloud<T>::Ptr> &clusters) = 0;

};

#endif
