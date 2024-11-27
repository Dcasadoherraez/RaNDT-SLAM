#include "radar_preprocessing/radar_preprocessor.h"

namespace rc
{
  namespace navigation
  {
    namespace ndt
    {

      void RadarPreprocessor::initialize(const ClusteringType &clustering_type, RadarPreprocessorParameters parameters, const Eigen::Affine3f &initial_transform_radar_baselink)
      {
        parameters_ = parameters;
        initial_transform_radar_baselink_ = initial_transform_radar_baselink;
        min_distance_ = parameters_.min_range;
        max_distance_ = parameters_.max_range;
        min_intensity_ = parameters_.min_intensity;
        clustering_type_ = clustering_type;
        switch (clustering_type)
        {
        // simplest case, others were discarded
        case ClusteringType::Grid:
          std::cout << "created Grid clustering \n";
          _cluster_generator.reset(new Grid());
          break;

        // here, other clustering points may be used
        default:
          std::cout << "No valid clustering algorithm!\n";
          break;
        }
        _cluster_generator->setMaxRange(parameters_.max_range);
      }

      void RadarPreprocessor::processScan(const sensor_msgs::PointCloud2::ConstPtr &cloud_in, std::vector<pcl::PointCloud<pcl::PointXYZI>> &clusters, std::vector<std::vector<std::pair<double, double>>> &polar_points, std::vector<std::tuple<double, double, double>> &max_detections)
      {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        std::vector<std::pair<double, double>> all_polar_points;                // polar representation of the points for pNDT
        filterScan(cloud_in, filtered_cloud, all_polar_points, max_detections); // get smaller scan
        std::vector<int> labels;
        labels.resize(filtered_cloud->size());
        _cluster_generator->cluster(parameters_.n_clusters, filtered_cloud, labels);                       // write labels to each points
        _cluster_generator->labelClouds(filtered_cloud, all_polar_points, labels, clusters, polar_points); // generate individual point clouds and polar points for each cluster
        for (size_t i = 0; i < clusters.size(); i++)
        {
          pcl_conversions::toPCL(cloud_in->header, clusters[i].header); // the points were already converted into the base frame, we have to do adjust the headers accordingly
          clusters[i].header.frame_id = parameters_.base_frame;
        }
        filtered_cloud->header = clusters[0].header;
      }

      void RadarPreprocessor::filterScan(const sensor_msgs::PointCloud2::ConstPtr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud, std::vector<std::pair<double, double>> &polar_point, std::vector<std::tuple<double, double, double>> &max_detections)
      {
        const int point_cloud_size = cloud_in->height * cloud_in->width;
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud_not_transformed(new pcl::PointCloud<pcl::PointXYZI>());      // pcl cloud of message
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_not_transformed(new pcl::PointCloud<pcl::PointXYZI>()); // filtered cloud in sensor frame
        raw_cloud_not_transformed->reserve(point_cloud_size);
        filtered_cloud_not_transformed->reserve(point_cloud_size);
        pcl::fromROSMsg(*cloud_in, *raw_cloud_not_transformed); // write into pcl cloud

        for (size_t j = 0; j < raw_cloud_not_transformed->size(); j++)
        {
          const float dist = std::hypot(raw_cloud_not_transformed->at(j).x, raw_cloud_not_transformed->at(j).y);
          const float angle = std::atan2(raw_cloud_not_transformed->at(j).y, raw_cloud_not_transformed->at(j).x);
          const float intensity = raw_cloud_not_transformed->at(j).intensity;
          // if (dist > min_distance_ && dist < max_distance_)
          // {
          auto point_no_z = raw_cloud_not_transformed->at(j);
          point_no_z.z = 0;
          filtered_cloud_not_transformed->push_back(point_no_z);
          polar_point.emplace_back(std::make_pair(angle, dist));
          // }
        }

        // for visualization
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        debug_cloud = filtered_cloud_not_transformed;
        debug_cloud->header.frame_id = parameters_.sensor_frame;
        pcl::transformPointCloud(*filtered_cloud_not_transformed, *filtered_cloud, initial_transform_radar_baselink_);
      }

      void RadarPreprocessor::splitClusters(std::vector<pcl::PointCloud<pcl::PointXYZI>> &clusters, std::vector<std::vector<std::pair<double, double>>> &polar_points)
      {
        std::vector<pcl::PointCloud<pcl::PointXYZI>> new_clusters;
        std::vector<std::vector<std::pair<double, double>>> new_polar_points;
        std::vector<int> labels;
        for (int i = 0; i < clusters.size(); i++)
        {
          labels.clear();
          labels.resize(clusters[i].size());
          pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_ptr(new pcl::PointCloud<pcl::PointXYZI>(clusters[i]));
          std::vector<pcl::PointCloud<pcl::PointXYZI>> new_subclusters;
          std::vector<std::vector<std::pair<double, double>>> new_sub_polar_points;
          if (clustering_type_ == ClusteringType::Grid)
          {
            _cluster_generator->splitCluster(cluster_ptr, parameters_.n_clusters * 4, labels);
          }
          else
          {
            _cluster_generator->splitCluster(cluster_ptr, 4, labels);
          }
          _cluster_generator->labelClouds(cluster_ptr, polar_points[i], labels, new_subclusters, new_sub_polar_points);
          new_clusters.insert(new_clusters.end(), new_subclusters.begin(), new_subclusters.end());
          new_polar_points.insert(new_polar_points.end(), new_sub_polar_points.begin(), new_sub_polar_points.end());
        }
        clusters = new_clusters;
      }

      // to generate vectors for each individual cluster
      void ClusterGenerator::labelClouds(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                         const std::vector<std::pair<double, double>> &polar_point,
                                         const std::vector<int> &labels,
                                         std::vector<pcl::PointCloud<pcl::PointXYZI>> &labeled_clouds,
                                         std::vector<std::vector<std::pair<double, double>>> &polar_points)
      {
        std::vector<int> labels_2_sort = labels;
        std::sort(labels_2_sort.begin(), labels_2_sort.end());
        int n_clusters = std::unique(labels_2_sort.begin(), labels_2_sort.end()) - labels_2_sort.begin();
        std::map<int, int> signed_index_2_unsigned;
        for (size_t i = 0; i < n_clusters; i++)
        {
          signed_index_2_unsigned[labels_2_sort[i]] = i;
        }
        labeled_clouds.resize(n_clusters);
        polar_points.resize(n_clusters);
        for (size_t i = 0; i < cloud->size(); i++)
        {
          labeled_clouds[signed_index_2_unsigned[labels[i]]].push_back(cloud->at(i));
          polar_points[signed_index_2_unsigned[labels[i]]].push_back(polar_point[i]);
        }
      }

      void ClusterGenerator::splitCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const int &n_new_clusters, std::vector<int> &labels)
      {
        cluster(n_new_clusters, cloud, labels);
      }
    }
  }
}