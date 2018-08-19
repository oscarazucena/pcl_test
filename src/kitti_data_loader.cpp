#include "kitti_data_loader.h"
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <iostream>
#include <sstream>

std::map<int, std::string> KEYS =
{
	{ 0,"lat" },
	{ 1,"lon" },
	{ 2,"alt" },
	{ 3,"roll" },
	{ 4,"pitch" },
	{ 5,"yaw" },
	{ 6,"vn" },
	{ 7,"ve" },
	{ 8,"vf" },
	{ 9,"vl" },
	{ 10,"vu" },
	{11,"ax" },
	{ 12,"ay" },
	{ 12,"ay" },
	{ 14,"af" },
	{ 15,"al" },
	{ 16,"au" },
	{ 17,"wx" },
	{ 18,"wy" },
	{ 19,"wz" },
	{ 20,"wf" },
	{ 21,"wl" },
	{ 22,"wu" },
	{ 23,"pos_accuracy" },
	{ 24,"vel_accuracy" },
	{ 25,"navstat" },
	{ 26,"numsats" },
	{ 27,"posmode" },
	{ 28,"velmode" },
	{ 29,"orimode" }
};

KITTIDataLoader::KITTIDataLoader(std::string file_path_in) :
    file_path(file_path_in)
{

}

PointXZYR_Cloud_Ptr KITTIDataLoader::run()
{
    if(loadData())
    {
        return  cloud_;
    }
	std::cout << "data not loaded" << std::endl;

    return nullptr;
}

bool KITTIDataLoader::loadData()
{
    cloud_.reset(new PointXZYR_Cloud);
    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));
    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    // load point cloud
    FILE *stream;

    stream = fopen (file_path.c_str(),"rb");
	if (!stream)
	{
		std::cout << "stream empty" << std::endl;
		return false;
	}
    //reader.read(reinterpret_cast<char*>(data),num);
    num = fread(data,sizeof(float),num,stream)/4;
    for (int32_t i=0; i<num; i++)
    {
        PointXYZR point;
        point.x = *px;
        point.y = *py;
        point.z = *pz;
        point.r = *pr;
        cloud_->push_back(point);
        px+=4; py+=4; pz+=4; pr+=4;
    }

    return true;
}

KITTIIMUGPSData::KITTIIMUGPSData(std::string file_path_in) : file_path(file_path_in)
{
}

std::map<std::string, float> KITTIIMUGPSData::run()
{
	if (loadData())
	{
		return  map;
	}
	std::cout << "data not loaded" << std::endl;

	return map;
}

bool KITTIIMUGPSData::loadData()
{

	
	std::ifstream f(file_path.c_str());
	std::string raw;
	
	int count = 0;
	while (std::getline(f, raw, ' '))
	{
		float num = atoi(raw.c_str());
		map[KEYS[count]] = num;
		count++;
	}

	return true;
}

KITTITimeStampData::KITTITimeStampData(std::string file_path_in) : file_path(file_path_in)
{
}

std::vector<std::tm> KITTITimeStampData::run()
{
	if (loadData())
	{
		return  map;
	}
	std::cout << "data not loaded" << std::endl;

	return map;
}

bool KITTITimeStampData::loadData()
{
	return false;
}
