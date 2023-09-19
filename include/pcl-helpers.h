#ifndef PCL_HELPERS_H
#define PCL_HELPERS_H
#include "common-includes.h"


namespace pclHelpers{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Vec3DToPointCloudXYZRGB(vector<cv::Point3f> points_3f, vector<cv::Point2f> image_coordinates, cv::Mat colorImage);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Vec3DToPointCloudXYZRGB(vector<cv::Point3d> points_3d, vector<cv::Point2d> image_coordinates, cv::Mat colorImage);
    void registerFirstPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr first_pc);
    void registerCurrentPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pc);
    void performICP(cv::Mat currentRot, cv::Mat currentPos, cv::Mat& outPos, cv::Mat& outRot);



}
#endif //PCL_HELPERS_H