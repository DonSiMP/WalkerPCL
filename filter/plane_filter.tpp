template <class T>
PlaneFilter<T>::PlaneFilter(bool optimize_coefficients, float threshold) {
    segmentator.setModelType(pcl::SACMODEL_PLANE);
    segmentator.setMethodType(pcl::SAC_RANSAC);

    segmentator.setOptimizeCoefficients(optimize_coefficients);
    segmentator.setDistanceThreshold(threshold);
}

template <class T>
PlaneFilter<T>::~PlaneFilter() {
}

template <class T>
void PlaneFilter<T>::filter(const typename pcl::PointCloud<T>::ConstPtr &input_cloud, typename pcl::PointCloud<T>::Ptr &filtered_cloud) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    segmentator.setInputCloud(input_cloud);
    segmentator.segment(*inliers, *coefficients);

#ifdef _DEBUG
    cout << "Plane coefficients: [" << coefficients->values[0] << ", " << coefficients->values[1] << ", "
         << coefficients->values[2] << ", " << coefficients->values[3] << "]" << std::endl;
#endif

    pcl::ExtractIndices<T> cloud_extractor(new pcl::ExtractIndices<T>);
    cloud_extractor.setInputCloud(input_cloud);
    cloud_extractor.setIndices(inliers);
    cloud_extractor.setNegative(true);

    cloud_extractor.filter(*filtered_cloud);
}
