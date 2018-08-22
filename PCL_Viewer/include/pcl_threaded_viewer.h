#ifndef PCLTHREADEDVIEWER_H
#define PCLTHREADEDVIEWER_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>

#include <thread>
#include <mutex>

#include <pcl_text_info.h>


#include <point_xyzr.h>

class PLCVisualizerWorker
{
public:

    PLCVisualizerWorker():stopped_(false), quit_(false)
    {

    }

    ~PLCVisualizerWorker()
    {
        stop();
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

    void addText(std::string key, const PCLTextInfo &info)
    {
        text_mutex.lock();
        texts_[key] = info;
        text_mutex.unlock();
    }

    void stop()
    {
        quit_= true;
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

                if(quit_)
                {
                    viewer.close();
                }
                else
                {
                    viewer.spinOnce(100);
                }
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
    bool quit_;
    std::thread thread;
};

#endif // PCLTHREADEDVIEWER_H
