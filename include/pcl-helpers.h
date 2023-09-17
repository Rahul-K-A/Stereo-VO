#ifndef PCL_HELPERS_H
#define PCL_HELPERS_H
#include "common-includes.h"


using namespace std;

namespace pclHelpers{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Vec3DToPointCloudXYZRGB(vector<cv::Point3d> points_3d, vector<cv::Point2f> image_coordinates, cv::Mat colorImage);
    void registerFirstPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr first_pc);
    void registerCurrentPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pc);
    cv::Mat performICP();

}
#endif //PCL_HELPERS_H