#include "pcl-helpers.h"
#include "cv-helpers.h"
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>

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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclHelpers::Vec3DToPointCloudXYZRGB(vector<cv::Point3d> points_3d, vector<cv::Point2d> image_coordinates, cv::Mat colorImage)
{
    //Convert Point3D into PointXYZRGB
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
        basic_point.x = (float)points_3d[i].x;
        basic_point.y = (float)points_3d[i].y;
        basic_point.z = (float)points_3d[i].z;
        cv::Point2f myPoint((float)image_coordinates[i].x, (float)image_coordinates[i].y);
        cv::Vec4b color = colorImage.at<cv::Vec4b>(image_coordinates[i]);
        basic_point.b = color[0];
        basic_point.g = color[1];
        basic_point.r = color[2];
        //cout<<"Basic point: "<< basic_point << endl; 
        point_cloud_ptr->points.push_back(basic_point);
    }

    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    //Since the point cloud is sparse, remove all the NaN values automatically set by PCL
    vector<int> indices;
    point_cloud_ptr->is_dense = false;
    pcl::removeNaNFromPointCloud(*point_cloud_ptr,*point_cloud_ptr_filtered, indices);
    // pcl::io::savePCDFileASCII("first_pre_filter.pcd",*point_cloud_ptr);
    // pcl::io::savePCDFileASCII("first.pcd", *point_cloud_ptr_filtered);
    return point_cloud_ptr_filtered;
}

/// @brief Register the first point cloud for setting up ICP
/// @param first_pc Input point cloud
void pclHelpers::registerFirstPointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr first_pc )
{
    previousPC = first_pc;
    currentPC = first_pc;
}

/// @brief Register a point cloud for setting up ICP
/// @param current_pc Input point cloud
void pclHelpers::registerCurrentPointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pc )
{
    previousPC = currentPC;
    currentPC = current_pc;
}

/// @brief Performs iterative closest point with the two most recently registered point clouds
/// @return OpenCV 4x4 pose matrix of type doubke

 /// @brief
 /// @param currentRot 
 /// @param currentPos 
 /// @param outPose 
 /// @param outRot 
 /// @return 
 void pclHelpers::performICP(cv::Mat currentRot, cv::Mat currentPos, cv::Mat& outPos, cv::Mat& outRot)
{
    #if 0
    pcl::PointCloud<pcl::PointXYZRGB> cc;
    //Set up ICP
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource( previousPC );
    icp.setInputTarget( currentPC);
    icp.setRANSACIterations(200);
    icp.setMaximumIterations(200);
    icp.setMaxCorrespondenceDistance(0.5f);
    icp.setTransformationEpsilon (1e-9);
    pcl::PointCloud<pcl::PointXYZRGB> Final;
    icp.align(*previousPC);
    //Get output transform
    Eigen::Matrix4f finalTransform = icp.getFinalTransformation();
    Eigen::Matrix4d finalTd = finalTransform.cast <double> ();
    cv:: Mat finalT;
    Eigen::Matrix4d converted;
    cv::eigen2cv(finalTd, finalT);
    cvHelpers::decompose4x4Pose(converted,bRot,bPos);

    //Get final transform
    cv::Mat currPose = cvHelpers::convertTo4x4Pose(currentRot, currentPos);
    cout<<"curr Pose :\n" << currPose << endl;
    cv::Mat finalPose = currPose * finalT;
    cv::cv2eigen(finalPose, converted);
    cout << "converted: \n" << converted << endl;
    cvHelpers::decompose4x4Pose(finalPose, outRot, outPos);
    cout<<"OutRot \n"<<outRot<<endl;
    cout<<"OutPos \n"<<outPos<<endl;

    cout<<"\n\n\n";
    //cout << finalT << endl;
    *combinedPC += *previousPC;

    pcl::io::savePCDFileASCII("final.pcd", *combinedPC);
    #endif
}