#ifndef PCL_HELPERS_H
#define PCL_HELPERS_H
#include "common-includes.h"

namespace pclHelpers{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Vec3DToPointCloudXYZRGB(vector<Point3d> points_3d, vector<Point2d> image_coordinates, cv::Mat colorImage);
    
}
#endif //PCL_HELPERS_H