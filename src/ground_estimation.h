#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

/**
 * @brief The PointXYZGround struct containing x,y,z, sx and sy (slope data)
 */
struct PointXYZGround
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float sx;
  float sy;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZGround,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, sx, sx)
                                   (float, sy, sy)
)

typedef pcl::PointCloud<PointXYZGround> PointXZYGround_Cloud;
typedef pcl::PointCloud<PointXYZGround>::Ptr PointXZYRGround_Cloud_Ptr;
