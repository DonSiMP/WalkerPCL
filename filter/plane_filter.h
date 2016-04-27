#ifndef _PLANE_FILTER_H_
#define _PLANE_FILTER_H_

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include "filter.h"

/*
 *  Template class that can be used for filtering the planes in a point cloud.
 */
template <class T>
class PlaneFilter: public Filter<T> {
public:
    // Creator. The object will filter all planes found in the input cloud. Set whether plane coeeficients should be refined and the allowed threshold distance from the plane.
    PlaneFilter(bool optimize_coefficients, float threshold);

    // Destructor.
    ~PlaneFilter();

    // Actual function for downsampling the input cloud.
    void filter(const typename pcl::PointCloud<T>::ConstPtr &input_cloud, typename pcl::PointCloud<T>::Ptr &filtered_cloud);

private:
    pcl::SACSegmentation<T> segmentator;

};

#include "plane_filter.tpp"

#endif
