#include "pcl-helpers.h"
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPC = nullptr;
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr previousPC  = nullptr;
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr combinedPC(new pcl::PointCloud<pcl::PointXYZRGB>);


bool isValidP3d(cv::Point3d point)
{
    bool valid = true;
    valid = valid && !isnan(point.x) && !isinf(point.x);
    valid = valid && !isnan(point.y) && !isinf(point.y);
    valid = valid && !isnan(point.z) && !isinf(point.z);
    return valid;
};

/// @brief Converts a vector of cv::Point3d to XYZRGB Point cloud
/// @param points_3d Input vector of cv::Point3d
/// @param image_coordinates Input cv::Point2d image coordinates corresponding to points_3d
/// @param colorImage Input BGR formatted cv::Mat used for sampling color
/// @return Pointer to point cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclHelpers::Vec3DToPointCloudXYZRGB(vector<cv::Point3d> points_3d, vector<cv::Point2f> image_coordinates, cv::Mat colorImage)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);//(new pcl::pointcloud<pcl::pointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);//(new pcl::pointcloud<pcl::pointXYZ>);
    for(uint16_t i = 0; i < points_3d.size(); i++)
    {
        if(!isValidP3d(points_3d[i]))
        {
            cout<<" Rejected point at " << i << " !\n";
            continue;
        }
        pcl::PointXYZRGB basic_point;
        basic_point.x = points_3d[i].x;
        basic_point.y = points_3d[i].y;
        basic_point.z = points_3d[i].z;
        cv::Vec3b color = colorImage.at<cv::Vec3b>(image_coordinates[i]);
        basic_point.b = color[0];
        basic_point.g = color[1];
        basic_point.r = color[2];
        point_cloud_ptr->points.push_back(basic_point);
    }

    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    vector<int> indices;
    point_cloud_ptr->is_dense = false;
    pcl::removeNaNFromPointCloud(*point_cloud_ptr,*point_cloud_ptr_filtered, indices);
    point_cloud_ptr.reset();
    return point_cloud_ptr_filtered;
}

void pclHelpers::registerFirstPointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr first_pc )
{
    previousPC = first_pc;
    currentPC = first_pc;
}

void pclHelpers::registerCurrentPointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pc )
{
    previousPC = currentPC;
    currentPC = current_pc;
}

cv::Mat pclHelpers::performICP()
{
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource( previousPC );
    icp.setInputTarget( currentPC);
    pcl::PointCloud<pcl::PointXYZRGB> Final;
    icp.align(Final);
    Eigen::Matrix4f finalTransform = icp.getFinalTransformation();
    cv:: Mat finalT;
    cv::eigen2cv(finalTransform, finalT);
    cout << finalT << endl;
    *combinedPC += Final;
    return finalT;
}