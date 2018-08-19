#define PCL_NO_PRECOMPILE
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "kitti_data_loader.h"



int main (int argc, char** argv)
{
    pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");

  // Read in the cloud data
  //pcl::PCDReader reader;
  pcl::PointCloud<PointXYZR>::Ptr cloud_f (new pcl::PointCloud<PointXYZR>);
  //reader.read ("table_scene_lms400.pcd", *cloud);
  KITTIDataLoader loader{"C:\\pcl_workspace\\KITTI\\2011_09_26\\2011_09_26_drive_0001_sync\\velodyne_points\\data\\0000000001.bin"};
   pcl::PointCloud<PointXYZR>::Ptr cloud = loader.run();
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<PointXYZR> vg;
  pcl::PointCloud<PointXYZR>::Ptr cloud_filtered (new pcl::PointCloud<PointXYZR>);
  vg.setInputCloud (cloud);
  double  leaf_l =  1.0;
  vg.setLeafSize (leaf_l, leaf_l, leaf_l);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<PointXYZR> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<PointXYZR>::Ptr cloud_plane (new pcl::PointCloud<PointXYZR> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (.25);

  int i=0, nr_points = (int) cloud->points.size ();
  while (cloud->points.size () > 0.4 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointXYZR> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointXYZR>::Ptr tree (new pcl::search::KdTree<PointXYZR>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointXYZR> ec;
  ec.setClusterTolerance (0.5); // 20cm
  ec.setMinClusterSize (20);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointXYZR>::Ptr cloud_cluster (new pcl::PointCloud<PointXYZR>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";

    pcl::visualization::PointCloudColorHandlerRandom<PointXYZR> cloud_color_handler (cloud_cluster);
    viewer.addPointCloud<PointXYZR> (cloud_cluster, cloud_color_handler, ss.str().c_str());
    writer.write<PointXYZR> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  //viewer.addPointCloud<PointXYZR> (cloud, "cloud");
  pcl::visualization::PointCloudColorHandlerCustom<PointXYZR> plane_color_handler (cloud_plane, 255, 0, 255);
  viewer.addPointCloud<PointXYZR> (cloud_plane, plane_color_handler, "cloud_plane");
  pcl::visualization::PointCloudColorHandlerCustom<PointXYZR> off_scene_model_color_handler (cloud_filtered, 255, 255, 255);
  viewer.addPointCloud<PointXYZR> (cloud_filtered, off_scene_model_color_handler, "cloud_filtered");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_filtered");
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");


  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return (0);
}
