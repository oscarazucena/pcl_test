#define PCL_NO_PRECOMPILE
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>

#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <chrono>



#include "kitti_data_loader.h"

//algorithm parameters
float model_ss_ (0.01f);
float scene_ss_ (0.01f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);


using namespace std;
using namespace std::literals::chrono_literals;

typedef PointXYZR PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

const std::string field_id{"x"};

void loadKITTICloud(std::string file_name);

std::vector<std::string> getListFile(std::string path)
{
    std::vector<std::string> files;

	if (!path.empty())
	{
		namespace fs = boost::filesystem;

		fs::path apk_path(path);
		fs::recursive_directory_iterator end;

		for (fs::recursive_directory_iterator i(apk_path); i != end; ++i)
		{
			const fs::path cp = (*i);
			files.push_back(cp.string());
		}
	}
	return files;
 
}


int user_data;


pcl::PointCloud<PointXYZR>::Ptr cloud (new pcl::PointCloud<PointXYZR>);
void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    PointXYZR o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
    
}

std::queue<PointXZYR_Cloud_Ptr> cloud_queue;
std::queue<pcl::PointCloud<pcl::Normal>::Ptr> normals_queue;
std::mutex cloud_queue_mutex;
std::condition_variable cloud_ready_conditional;
bool cloud_ready = false;

//normal stuff
int display_points = 100;
double  normal_scale = 10;
double search_radius = 5;
int search_count = 10;
bool show_normals = true;
void 
viewerPsycho (std::string file_name)
{
    loadKITTICloud(file_name);
}

pcl::PointCloud<pcl::Normal>::Ptr findNormals(PointXZYR_Cloud_Ptr cloud)
{
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<PointXYZR, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointXYZR>::Ptr tree (new pcl::search::KdTree<PointXYZR> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 1m
    ne.setKSearch(search_count);

    // Compute the features
    ne.compute (*cloud_normals);

    return cloud_normals;
}

void addCloud(PointXZYR_Cloud_Ptr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    if(show_normals)
    {
        normals =   findNormals(cloud);
    }

    {
        std::lock_guard<std::mutex> lock(cloud_queue_mutex);
        cloud_queue.push(cloud);
        if(show_normals)
        {
            normals_queue.push(normals);
        }
        cloud_ready = true;
    }
    cloud_ready_conditional.notify_one();
}

void loadKITTICloud(string file_name)
{
    KITTIDataLoader loader{file_name};
    addCloud(loader.run());
}

double
computeCloudResolution (const pcl::PointCloud<PointXYZR>::ConstPtr &cloud)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<PointXYZR> tree;
    tree.setInputCloud (cloud);

    for (size_t i = 0; i < cloud->size (); ++i)
    {
        if (! pcl_isfinite ((*cloud)[i].x))
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}

void compouterKeyPoints( pcl::PointCloud<PointType>::Ptr model,
                         pcl::PointCloud<PointType>::Ptr model_keypoints,
                         pcl::PointCloud<PointType>::Ptr scene,
                         pcl::PointCloud<PointType>::Ptr scene_keypoints)
{
    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (model);
    uniform_sampling.setRadiusSearch (model_ss_);
    uniform_sampling.filter (*model_keypoints);
    std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

    uniform_sampling.setInputCloud (scene);
    uniform_sampling.setRadiusSearch (scene_ss_);
    uniform_sampling.filter (*scene_keypoints);
    std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;
}

void getDescriptors(pcl::PointCloud<PointType>::Ptr model,
                    pcl::PointCloud<PointType>::Ptr model_keypoints ,
                    pcl::PointCloud<PointType>::Ptr scene,
                    pcl::PointCloud<PointType>::Ptr scene_keypoints,
                    pcl::PointCloud<NormalType>::Ptr model_normals,
                    pcl::PointCloud<NormalType>::Ptr scene_normals ,
                    pcl::PointCloud<DescriptorType>::Ptr model_descriptors,
                    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors)
{
    //
    //  Compute Descriptor for keypoints
    //
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch (descr_rad_);

    descr_est.setInputCloud (model_keypoints);
    descr_est.setInputNormals (model_normals);
    descr_est.setSearchSurface (model);
    descr_est.compute (*model_descriptors);

    descr_est.setInputCloud (scene_keypoints);
    descr_est.setInputNormals (scene_normals);
    descr_est.setSearchSurface (scene);
    descr_est.compute (*scene_descriptors);
}

void findCorresponance(pcl::CorrespondencesPtr model_scene_corrs,
                       pcl::PointCloud<DescriptorType>::Ptr model_descriptors,
                       pcl::PointCloud<DescriptorType>::Ptr scene_descriptors)
{
    //
    //  Find Model-Scene Correspondences with KdTree
    //
    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud (model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (size_t i = 0; i < scene_descriptors->size (); ++i)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back (corr);
        }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

}

void clustering(     std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &rototranslations,
                     std::vector<pcl::Correspondences> &clustered_corrs,
                     pcl::CorrespondencesPtr model_scene_corrs,
                     pcl::PointCloud<PointType>::Ptr model_keypoints ,
                     pcl::PointCloud<PointType>::Ptr scene_keypoints
                     )
{
    //
    //  Actual Clustering
    //
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);

}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<PointXYZR>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerGenericField<PointXYZR> color_handler= pcl::visualization::PointCloudColorHandlerGenericField<PointXYZR> (cloud, field_id);
    viewer->addPointCloud<PointXYZR> (cloud, color_handler, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer->addCoordinateSystem (1.0);
    if(show_normals)
    {
        viewer->addPointCloudNormals<PointXYZR, pcl::Normal> (cloud, normals, display_points, normal_scale, "normals");
    }
    viewer->initCameraParameters ();
    return (viewer);
}

std::atomic<bool> stop = {false};
void loadData()
{
	std::cout << __FUNCDNAME__ << std::endl;
    std::string path {"C:\pcl_workspace\KITTI\2011_09_26\2011_09_26_drive_0001_sync\velodyne_points\data"};
    std::vector<std::string> files = getListFile(path);
    std::sort(files.begin(),files.end());
    int size = files.size();
    int count = 0;
    while(!stop)
    {
        viewerPsycho(files[count]);
        count = ++count%size;
    }
	std::cout << __FUNCDNAME__ << std::endl;
}

int 
main ()
{    

	std::cout << __FUNCDNAME__ << std::endl;

    pcl::PointCloud<PointType>::Ptr model;
    pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());
    //
    //  Find Model-Scene Correspondences with KdTree
    //
	std::cout << "loading data" << std::endl;

    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
    KITTIDataLoader loader{"C:\\pcl_workspace\\KITTI\\2011_09_26\\2011_09_26_drive_0001_sync\\velodyne_points\\data\\0000000000.bin"};
    model = loader.run();
	std::cout << "data loaded" << std::endl;

    pcl::copyPointCloud(*model,*scene);
	std::cout << "Copy data" << std::endl;

    float resolution = static_cast<float> (computeCloudResolution (model));
	std::cout << "computeCloudResolution" << std::endl;

    if (resolution != 0.0f)
    {
        model_ss_   *= resolution;
        scene_ss_   *= resolution;
        rf_rad_     *= resolution;
        descr_rad_  *= resolution;
        cg_size_    *= resolution;
    }

    std::cout << "Model resolution:       " << resolution << std::endl;
    std::cout << "Model sampling size:    " << model_ss_ << std::endl;
    std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
    std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
    std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
    std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;

    //
     //  Compute Normals
     //
	std::cout << "compute normals" << std::endl;

     pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
     norm_est.setKSearch (10);
     norm_est.setInputCloud (model);
     norm_est.compute (*model_normals);

     norm_est.setInputCloud (scene);
     norm_est.compute (*scene_normals);

    compouterKeyPoints(model, model_keypoints, scene, scene_keypoints);
    getDescriptors(model, model_keypoints , scene, scene_keypoints, model_normals, scene_normals , model_descriptors, scene_descriptors);
    findCorresponance( model_scene_corrs, model_descriptors, scene_descriptors);

    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    clustering(rototranslations, clustered_corrs, model_scene_corrs, model_keypoints, scene_keypoints);

    //
    //  Visualization
    //
	std::cout << "Visualization" << std::endl;

    bool show_correspondences_ = true;
    bool show_keypoints_ = true;

    pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
    viewer.addPointCloud<PointType> (scene, "scene_cloud");

    pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

    if (show_correspondences_ || show_keypoints_)
    {
      //  We are translating the model so that it doesn't end in the middle of the scene representation
      pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
      pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

      pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
      viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
    }

    if (show_keypoints_)
    {
      pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
      viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

      pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
      viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
    }

    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
      pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
      pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

      std::stringstream ss_cloud;
      ss_cloud << "instance" << i;

      pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
      viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

      if (show_correspondences_)
      {
        for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
        {
          std::stringstream ss_line;
          ss_line << "correspondence_line" << i << "_" << j;
          PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
          PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

          //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
          viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
        }
      }
    }

    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }

    return (0);
//    pcl::PointCloud<pcl::Normal>::Ptr normals;
//    if(show_normals)
//    {
//        normals = findNormals(cloud);
//    }

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(cloud, normals);


//    //std::thread load_files(loadData);
//    std::cout << "1" << std::endl;

//    viewer->resetCamera();

//    bool loaded = false;
//    //--------------------
//    // -----Main loop-----
//    //--------------------
//    while (!viewer->wasStopped ())
//    {
//        {
//            std::unique_lock<std::mutex> lock(cloud_queue_mutex);
//            cloud_ready_conditional.wait_for(lock, 100ms, []{return (cloud_ready || (cloud_queue.size() > 0));});

//            if(cloud_queue.size() > 0)
//            {
//                cloud = cloud_queue.front();
//                if(show_normals)
//                {

//                    normals = normals_queue.front();
//                    normals_queue.pop();
//                }
//                cloud_queue.pop();
//                loaded = true;
//            }
//            cloud_ready = false;
//        }
//        if(loaded)
//        {
//            viewer->removePointCloud("cloud");
//            pcl::visualization::PointCloudColorHandlerGenericField<PointXYZR> color_handler= pcl::visualization::PointCloudColorHandlerGenericField<PointXYZR> (cloud, field_id);
//            viewer->addPointCloud<PointXYZR> (cloud, color_handler, "cloud");
//            if(show_normals)
//            {
//                viewer->removePointCloud("normals");
//                viewer->addPointCloudNormals<PointXYZR, pcl::Normal> (cloud, normals, display_points, normal_scale, "normals");
//            }
//            //viewer->addPointCloud(cloud,"cloud");
//            //viewer->removePointCloud("cloud");
//            loaded = false;
//        }
//        viewer->spinOnce (100);
//        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }
//    stop = true;
//    //load_files.join();
//    return 0;
}
