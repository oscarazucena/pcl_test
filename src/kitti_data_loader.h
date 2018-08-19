#ifndef KITTIDATALOADER_H
#define KITTIDATALOADER_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <string>
#include "point_xyzr.h"
#include <boost/signals2.hpp>

class KITTITimeStampData
{
public:
	KITTITimeStampData(std::string file_path_in);
	std::vector<std::tm> run();
private:
	bool loadData();
	std::string file_path;
	std::vector<std::tm> map;
};

class KITTIIMUGPSData
{
public:
	KITTIIMUGPSData(std::string file_path_in);
	std::map<std::string, float> run();
private:
	bool loadData();
	std::string file_path;
	std::map<std::string, float> map;
};
class KITTIDataLoader
{
public:

    KITTIDataLoader(std::string file_path_in);
    PointXZYR_Cloud_Ptr run();

private:
    bool loadData();
    std::string file_path;
    PointXZYR_Cloud_Ptr cloud_;
};

#endif // KITTIDATALOADER_H
