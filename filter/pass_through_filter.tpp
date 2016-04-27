template <class T>
PassThroughFilter<T>::PassThroughFilter(const std::string &field_name, float min, float max) {
    pass_through.setFilterFieldName(field_name);
    pass_through.setFilterLimits(min, max);
    pass_through.setNegative(false);
}

template <class T>
PassThroughFilter<T>::~PassThroughFilter() {
}

template <class T>
void PassThroughFilter<T>::filter(const typename pcl::PointCloud<T>::ConstPtr &input_cloud, typename pcl::PointCloud<T>::Ptr &filtered_cloud) {
    pass_through.setInputCloud(input_cloud);
    pass_through.filter(*filtered_cloud);
}
