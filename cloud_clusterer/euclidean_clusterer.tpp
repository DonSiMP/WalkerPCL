template <class T>
EuclideanClusterer<T>::EuclideanClusterer(float tolerance, unsigned int min_size, unsigned int max_size) {
    clusterer.setClusterTolerance(tolerance);
    clusterer.setMinClusterSize(min_size);
    clusterer.setMaxClusterSize(max_size);
}

template <class T>
EuclideanClusterer<T>::~EuclideanClusterer() {
}

template <class T>
void EuclideanClusterer<T>::cluster(const typename pcl::PointCloud<T>::ConstPtr &input_cloud, std::vector<typename pcl::PointCloud<T>::Ptr> &clusters) {

    /* Extract cluster indices */

    typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>);
    tree->setInputCloud(input_cloud);

    clusterer.setSearchMethod(tree);
    clusterer.setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    clusterer.extract(cluster_indices);

#ifdef _DEBUG
    std::cout << cluster_indices.size() << " clusters detected" << std::endl;
#endif

    /* Extract point clouds using cluster indices */

    pcl::ExtractIndices<T> cloud_extractor(new pcl::ExtractIndices<T>);
    cloud_extractor.setInputCloud(input_cloud);
    for (std::vector<pcl::PointIndices>::const_iterator cluster_it = cluster_indices.begin(); cluster_it != cluster_indices.end(); ++cluster_it) {
        pcl::PointIndices::Ptr indices = boost::make_shared<pcl::PointIndices> (*cluster_it); //TODO, copy, it's not efficient, check nop_destructor alternative hack

        cloud_extractor.setIndices(indices);
        cloud_extractor.setNegative(false);

        typename pcl::PointCloud<T>::Ptr cluster(new pcl::PointCloud<T>);

        cloud_extractor.filter(*cluster);
        clusters.push_back(cluster);
    }
}
