#ifndef KITTIT_TIME_STAMP_DATA_H

#include <string>
#include <chrono>
#include <vector>

struct KITTITimeStamp
{
    std::tm tm;
    std::chrono::nanoseconds nanoseconds;
};

class KITTITimeStampData
{
public:
        KITTITimeStampData(std::string file_path_in);
    std::vector<KITTITimeStamp> run();
private:
        bool loadData();
        std::string file_path;
    std::vector<KITTITimeStamp> map;
};
#define KITTIT_TIME_STAMP_DATA_H

#endif // KITTIT_TIME_STAMP_DATA_H
