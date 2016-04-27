#ifndef _PASS_THROUGH_FILTER_H_
#define _PASS_THROUGH_FILTER_H_

#include <pcl/filters/passthrough.h>
#include "filter.h"

/*
 *  Template class that can be used for applying a passthrough filter to a given point cloud.
 */
template <class T>
class PassThroughFilter: public Filter<T> {
public:
    // Creator. The object will apply a passthrough filter to the field_name field from min to max to the input cloud.
    PassThroughFilter(const std::string &field_name, float min, float max);

    // Destructor.
    ~PassThroughFilter();

    // Actual function for filtering the input cloud.
    void filter(const typename pcl::PointCloud<T>::ConstPtr &input_cloud, typename pcl::PointCloud<T>::Ptr &filtered_cloud);

private:
    pcl::PassThrough<T> pass_through;

};

#include "pass_through_filter.tpp"

#endif
