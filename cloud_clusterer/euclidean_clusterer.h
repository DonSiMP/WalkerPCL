#ifndef _EUCLIDEAN_CLUSTERER_H_
#define _EUCLIDEAN_CLUSTERER_H_

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include "cloud_clusterer.h"

/*
 *  Template class that can be used for extracting the keypoints from a point cloud.
 */
template <class T>
class EuclideanClusterer: public CloudClusterer<T> {
public:
    // Creator. The object will break the input cloud into clusters of at least min_size points and at max max_size points, using the given tolerance value.
    EuclideanClusterer(float tolerance, unsigned int min_size, unsigned int max_size);

    // Destructor.
    ~EuclideanClusterer();

    // Actual function for segment the input cloud into clusters.
    void cluster(const typename pcl::PointCloud<T>::ConstPtr &input_cloud, std::vector<typename pcl::PointCloud<T>::Ptr> &clusters);

private:
    pcl::EuclideanClusterExtraction<T> clusterer;

};

#include "euclidean_clusterer.tpp"

#endif
