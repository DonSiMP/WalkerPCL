template <class T>
Downsampler<T>::Downsampler(float size_x, float size_y, float size_z) {
    downsampler.setLeafSize(size_x, size_y, size_z);
}

template <class T>
Downsampler<T>::~Downsampler() {
}

template <class T>
void Downsampler<T>::filter(const typename pcl::PointCloud<T>::ConstPtr &input_cloud, typename pcl::PointCloud<T>::Ptr &filtered_cloud) {
    downsampler.setInputCloud(input_cloud);
    downsampler.filter(*filtered_cloud);
}
