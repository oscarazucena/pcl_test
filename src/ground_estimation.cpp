#define PCL_NO_PRECOMPILE
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <thread>         // std::thread
#include <mutex>          // std::mutex
#include <chrono>

#include "kitti_data_loader.h"

#include <Eigen/Core>

#include "ground_estimation.h"
#include <Eigen/Dense>
#include <string>
using Ground = Eigen::Vector3f;
using GroundPlane = std::vector<Ground>;
using MasurementMat = Eigen::Vector3f;
using H = MasurementMat;
using cij = bool;
using C = std::vector<cij>;
using GP = std::vector<float>;
using F = Eigen::Matrix3f;

#define GET_INDEX(x,y,n) ((y)*(n)+(x))

float calculateHeightProbability(const PointXYZR p, const PointXYZGround Xi, const float sig_up_2, const float sig_down_2)
{
    Eigen::Vector3f hij(1.0f,Xi.x-p.x,Xi.y-p.y);
    Eigen::Vector3f xi(Xi.z, Xi.sx, Xi.sy);
    float dz = p.z - hij.transpose()*xi;
    float prob = 0.0;
    if(dz > 0)
    {
        prob = exp(-dz*dz/(2.0*sig_up_2));
    }
    else
    {
        prob = exp(-dz*dz/(2.0*sig_down_2));
    }
    return prob;
}

void calculateCloudProbability(PointXZYR_Cloud &cloud,const PointXZYGround_Cloud &Xi, float s1, float s2,float dl, float limit, int n)
{
    for (auto &point : cloud.points)
    {
        int nx = round((point.x+limit)/ dl);
        int ny = round((point.y+limit)/ dl);
        if(nx < 0 || nx >= n) continue;
        if(ny < 0 || ny >= n) continue;
        auto xi = Xi.points[GET_INDEX(nx,ny,n)];
        float pc = calculateHeightProbability(point,xi,s1,s2);
        //use r as the probability
        point.r = pc;
    }
}

void calculateNewXandP(PointXZYGround_Cloud &Xi_K_1, const PointXZYGround_Cloud &Xi_K, PointXZYGround_Cloud &Xi_K_m_1, const PointXZYR_Cloud &cloud, const PointXZYGround_Cloud &X, float dl, float limit, float alpha, float beta, float gamma, int n)
{
	for ( auto &xi : Xi_K_1.points)
	{
		xi.sx = 0;
		xi.sy = 0;
		xi.z = 0;
	}

	for (auto &point : cloud.points)
	{
		int nx = round((point.x + limit) / dl);
		int ny = round((point.y + limit) / dl);
		if (nx < 0 || nx >= n) continue;
		if (ny < 0 || ny >= n) continue;
		size_t index = GET_INDEX(nx, ny, n);
		auto xi = Xi_K.points[index];
		auto &xi_k_1 = Xi_K_1.points[index];
		Eigen::Vector3f hij(1.0f, xi.x - point.x, xi.y - point.y);
		Eigen::Vector3f result = alpha*point.r*point.z*hij;
		xi_k_1.z += result(0);
		xi_k_1.sx += result(1);
		xi_k_1.sy += result(2);
	}

	for (auto &xi_k_1 : Xi_K_1.points)
	{
		int nx = round((xi_k_1.x + limit) / dl);
		int ny = round((xi_k_1.y + limit) / dl);
		if (nx < 0 || nx >= n) continue;
		if (ny < 0 || ny >= n) continue;

		if (nx - 1 > 0)
		{
			F f;
			f << 1.0f, -dl, 0,
				0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 1.0f;
			auto xi_1 = Xi_K.points[GET_INDEX(nx-1,ny,n)];
			Eigen::Vector3f xi_k(xi_1.z, xi_1.sx, xi_1.sy);
			Eigen::Vector3f result = beta*f.transpose()*xi_k;
			xi_k_1.z += result(0);
			xi_k_1.sx += result(1);
			xi_k_1.sy += result(2);
		}
		if (nx + 1 < n)
		{
			F f;
			f << 1.0f, dl, 0,
				0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 1.0f;
			auto xi_1 = Xi_K.points[GET_INDEX(nx + 1, ny, n)];
			Eigen::Vector3f xi_k(xi_1.z, xi_1.sx, xi_1.sy);
			Eigen::Vector3f result = beta*f.transpose()*xi_k;
			xi_k_1.z += result(0);
			xi_k_1.sx += result(1);
			xi_k_1.sy += result(2);
		}

		if (ny - 1 > 0)
		{
			F f;
			f << 1.0f, 0, -dl,
				0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 1.0f;
			auto xi_1 = Xi_K.points[GET_INDEX(nx , ny - 1, n)];
			Eigen::Vector3f xi_k(xi_1.z, xi_1.sx, xi_1.sy);
			Eigen::Vector3f result = beta*f.transpose()*xi_k;
			xi_k_1.z += result(0);
			xi_k_1.sx += result(1);
			xi_k_1.sy += result(2);
		}
		if (ny + 1 < n)
		{
			F f;
			f << 1.0f, 0, dl,
				0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 1.0f;
			auto xi_1 = Xi_K.points[GET_INDEX(nx , ny + 1, n)];
			Eigen::Vector3f xi_k(xi_1.z, xi_1.sx, xi_1.sy);
			Eigen::Vector3f result = beta*f.transpose()*xi_k;
			xi_k_1.z += result(0);
			xi_k_1.sx += result(1);
			xi_k_1.sy += result(2);
		}

		auto xi_k_m1 = Xi_K_m_1.points[GET_INDEX(nx, ny, n)];
  		xi_k_1.z += gamma*xi_k_m1.z;
		xi_k_1.sx += gamma*xi_k_m1.sx;
		xi_k_1.sy += gamma*xi_k_m1.sy;
	}
}

struct PCLTextInfo
{
	PCLTextInfo()
	{

	}
	PCLTextInfo(std::string t, int xin, int yin)
	{
		text = t;
		x = xin;
		y = yin;
	}
	int x;
	int y;
	std::string text;
};

class PLCVisualizerWorker
{
public:

	PLCVisualizerWorker() :stopped_(false)
	{

	}

	~PLCVisualizerWorker() 
	{
		thread.join();
	}

	bool wasStopped()
	{
		return stopped_;
	}

	void addCloud(pcl::PointCloud<PointXYZR>::Ptr cloud, pcl::visualization::PointCloudColorHandlerCustom<PointXYZR>::Ptr color, std::string key)
	{
		mtx.lock();
		clouds_[key] = cloud;
		color_[key] = color;
		mtx.unlock();

	}

	void addText(std::string key, PCLTextInfo info)
	{
		text_mutex.lock();
		texts_[key] = info;
		text_mutex.unlock();
	}

	void run()
	{
		thread = std::thread([this] {
			pcl::visualization::PCLVisualizer viewer("Cloud Display");

			while (!viewer.wasStopped())
			{

				{
					mtx.lock();
					if (!clouds_.empty())
					{
						for (auto cloud_pair : clouds_)
						{

							if (!viewer.updatePointCloud<PointXYZR>(cloud_pair.second, *color_[cloud_pair.first], cloud_pair.first))
							{
								std::cout << "Add cloud: " << cloud_pair.first << std::endl;
								viewer.addPointCloud<PointXYZR>(cloud_pair.second, *color_[cloud_pair.first], cloud_pair.first);
							}
						}
					}
					clouds_.clear();
					color_.clear();
					mtx.unlock();
					text_mutex.lock();
					if(!texts_.empty())
					{
						for (auto text_pair : texts_)
						{
							if (!viewer.updateText(text_pair.second.text, text_pair.second.x, text_pair.second.y, text_pair.first))
							{
								viewer.addText(text_pair.second.text, text_pair.second.x, text_pair.second.y, text_pair.first);
							}
						}
					}
					texts_.clear();
					text_mutex.unlock();
				}
				viewer.spinOnce(100);
			}
			stopped_ = true;
		});
	}


private:
	std::map<std::string, pcl::PointCloud<PointXYZR>::Ptr> clouds_;
	std::map<std::string, pcl::visualization::PointCloudColorHandlerCustom<PointXYZR>::Ptr> color_;
	std::map<std::string, PCLTextInfo> texts_;
	std::mutex mtx;           // mutex for critical section
	std::mutex text_mutex;    //mutex to lock text addition
	bool stopped_;
	std::thread thread;
};

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

int main (int argc, char** argv)
{

    int nx = 100;
    int ny = 100;
    int N = nx*ny;
    float dl = .50;
    float x_limit = nx*dl/2;
    float y_limit = ny*dl/2;

    F f;
    f << 1.0f, -dl, -dl,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f;

    float sig_up_2 = .05*.05;
    float sig_down_2 = .5*.5;

    PointXZYGround_Cloud ground_cloud;
    ground_cloud.width    = nx;
    ground_cloud.height   = ny;
    ground_cloud.is_dense = true;
    ground_cloud.points.resize (nx * ny);
    PointXZYGround_Cloud X_cloud;
    X_cloud.width    = nx;
    X_cloud.height   = ny;
    X_cloud.is_dense = true;
    X_cloud.points.resize (nx * ny);
    PointXZYGround_Cloud P_cloud;
    P_cloud.width    = nx;
    P_cloud.height   = ny;
    P_cloud.is_dense = true;
    P_cloud.points.resize (nx * ny);
    pcl::PointCloud<PointXYZR> prob_cloud;
    for(int y = 0; y < ny;y++)
    {
        float ly = dl*y-y_limit;
        for(int x = 0; x < nx;x++)
        {
            float lx = dl*x-x_limit;
            PointXYZGround p{lx,ly,0.0,0.0,0.0};
            ground_cloud.points[GET_INDEX(x,y,nx)] = p;
            X_cloud.points[GET_INDEX(x,y,nx)] = p;
            P_cloud.points[GET_INDEX(x,y,nx)] = p;
        }
    }
	PLCVisualizerWorker worker;
	worker.run();
	auto files = getListFile("C:\\pcl_workspace\\KITTI\\2011_09_26\\2011_09_26_drive_0001_sync\\velodyne_points\\data\\");
	auto gps_files = getListFile("C:\\pcl_workspace\\KITTI\\2011_09_26\\2011_09_26_drive_0001_sync\\oxts\\data\\");
	int counter = 0;
	for (auto file : files)
	{
		// Read in the cloud data
		//pcl::PCDReader reader;
		pcl::PointCloud<PointXYZR>::Ptr cloud_f(new pcl::PointCloud<PointXYZR>);
		//reader.read ("table_scene_lms400.pcd", *cloud);
		KITTIDataLoader loader{ file };
		KITTIIMUGPSData gps{gps_files[counter++]};
		auto gps_map = gps.run();
		pcl::PointCloud<PointXYZR>::Ptr cloud = loader.run();
		pcl::PointCloud<PointXYZR>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZR>);
		pcl::PointCloud<PointXYZR>::Ptr cloud_remaining(new pcl::PointCloud<PointXYZR>);
		// Create the filtering object: filter data to 100x100 m area
		pcl::ConditionAnd<PointXYZR>::Ptr range_cond(new pcl::ConditionAnd<PointXYZR>());
		range_cond->addComparison(pcl::FieldComparison<PointXYZR>::Ptr(new pcl::FieldComparison<PointXYZR>("x", pcl::ComparisonOps::LT, x_limit)));
		range_cond->addComparison(pcl::FieldComparison<PointXYZR>::Ptr(new pcl::FieldComparison<PointXYZR>("x", pcl::ComparisonOps::GT, -x_limit)));
		range_cond->addComparison(pcl::FieldComparison<PointXYZR>::Ptr(new pcl::FieldComparison<PointXYZR>("y", pcl::ComparisonOps::LT, y_limit)));
		range_cond->addComparison(pcl::FieldComparison<PointXYZR>::Ptr(new pcl::FieldComparison<PointXYZR>("y", pcl::ComparisonOps::GT, -y_limit)));
		pcl::ConditionalRemoval<PointXYZR> range_filt;
		range_filt.setCondition(range_cond);
		range_filt.setInputCloud(cloud);
		range_filt.setKeepOrganized(false);

		// The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
		range_filt.filter(*cloud_filtered);
		// The indices_rem array indexes all points of cloud_in that have x smaller than 0.0 or larger than 1000.0
		//indices_rem = range_filt.getRemovedIndices ();

		//viewer.addPointCloud<PointXYZR> (cloud, "cloud");
		pcl::visualization::PointCloudColorHandlerCustom<PointXYZR>::Ptr plane_color_handler{ new pcl::visualization::PointCloudColorHandlerCustom<PointXYZR>(cloud, 255, 0, 255) };
		pcl::visualization::PointCloudColorHandlerCustom<PointXYZR>::Ptr off_scene_model_color_handler{ new pcl::visualization::PointCloudColorHandlerCustom<PointXYZR>(cloud_filtered, 0, 255, 255) };
		//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_filtered");
		//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
	   worker.addCloud(cloud, plane_color_handler,"cloud");
	  worker.addCloud(cloud_filtered, off_scene_model_color_handler, "cloud_filtered");
	  auto format = [](float data)
	  {
		  return std::to_string(data);
	  };
	  PCLTextInfo info = { file,0,0 };
	  PCLTextInfo lat = { format(gps_map["vf"]),0,20 };
	  PCLTextInfo lon = { format(gps_map["vl"]),0,40 };

	  worker.addText("File",info);
	  worker.addText("vf", lat);
	  worker.addText("vl", lon);
	  std::cout << "File: " << file << std::endl;
	  for (auto pair : gps_map)
	  {
		  std::cout << pair.first << " : " << pair.second << std::endl;
	  }
	}
	while (!worker.wasStopped())
	{
		std::this_thread::sleep_for(std::chrono::nanoseconds(1));
		continue;
	}
	std::cout << "Outside of loop" << std::endl;

    return (0);
}
