#include <kitti_time_stamp_data.h>

#include <iostream>
#include <sstream>
#include <time.h>
#include <fstream>

KITTITimeStampData::KITTITimeStampData(std::string file_path_in) : file_path(file_path_in)
{
}

std::vector<KITTITimeStamp> KITTITimeStampData::run()
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
    std::ifstream f(file_path.c_str());
    std::string raw;

    while (std::getline(f, raw, '\n'))
    {
        KITTITimeStamp stamp;
        std::tm tm;
        strptime(raw.c_str(),"%F %H:%M:%S",&stamp.tm);
        auto pos = raw.find('.');
        if(pos+1 < raw.size())
        {
            auto sub_string = raw.substr(pos+1, raw.size()-1);
            long nanoseconds = (long)::atof(sub_string.c_str());
            stamp.nanoseconds = std::chrono::nanoseconds(nanoseconds);
        }
        map.push_back(stamp);
    }

    return true;
}
