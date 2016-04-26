#ifndef _FILTER_H_
#define _FILTER_H_

#include <vector>
#include <pcl/point_cloud.h>

/*
 *  Template generic class that can be used for filtering a point cloud.
 */
template <class T>
class Filter {
public:
    // Destructor.
    virtual ~Filter() {}

    // Actual method for filtering the input cloud.
    virtual void filter(const typename pcl::PointCloud<T>::ConstPtr &input_cloud, typename pcl::PointCloud<T>::Ptr &filtered_cloud) = 0;

};

#endif
