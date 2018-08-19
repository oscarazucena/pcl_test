#ifndef POINT_XZYR_H
#define POINT_XZYR_H
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

/**
 * @brief The PointXYZR struct containing x,y,z, and r (reflectance data)
 */
struct PointXYZR
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float r;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZR,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, r, r)
)

typedef pcl::PointCloud<PointXYZR> PointXZYR_Cloud;
typedef pcl::PointCloud<PointXYZR>::Ptr PointXZYR_Cloud_Ptr;

#endif // POINT_XZYR_H
