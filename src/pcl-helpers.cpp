#include "pcl-helpers.h"


/// @brief Converts a vector of cv::Point3d to XYZRGB Point cloud
/// @param points_3d Input vector of cv::Point3d
/// @param image_coordinates Input cv::Point2d image coordinates corresponding to points_3d
/// @param colorImage Input BGR formatted cv::Mat used for sampling color
/// @return Pointer to point cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclHelpers::Vec3DToPointCloudXYZRGB(vector<Point3d> points_3d, vector<Point2d> image_coordinates, cv::Mat colorImage)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);//(new pcl::pointcloud<pcl::pointXYZ>);

    for(uint16_t i = 0; i < points_3d.size(); i++)
    {
        pcl::PointXYZRGB basic_point;
        basic_point.x = points_3d[i].x;
        basic_point.y = points_3d[i].y;
        basic_point.z = points_3d[i].z;
        Vec3b color = colorImage.at<Vec3b>(image_coordinates[i]);
        basic_point.b = color[0];
        basic_point.g = color[1];
        basic_point.r = color[2];

        point_cloud_ptr->points.push_back(basic_point);
    }
    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

    return point_cloud_ptr;
}