/* \author Geoffrey Biggs */

#include <iostream>
#include <thread>

#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/ximgproc/disparity_filter.hpp>
#include <opencv4/opencv2/xfeatures2d.hpp>
#include <iostream>
#include <string>


using namespace std::chrono_literals;
using namespace cv;
using namespace cv::ximgproc;
using namespace std;
using namespace cv::xfeatures2d;
// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}


pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


pcl::visualization::PCLVisualizer::Ptr customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


pcl::visualization::PCLVisualizer::Ptr normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


pcl::visualization::PCLVisualizer::Ptr shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  viewer->addLine<pcl::PointXYZRGB> ((*cloud)[0],
                                     (*cloud)[cloud->size() - 1], "line");
  viewer->addSphere ((*cloud)[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  viewer->addPlane (coeffs, "plane");
  coeffs.values.clear ();
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (5.0);
  viewer->addCone (coeffs, "cone");

  return (viewer);
}


pcl::visualization::PCLVisualizer::Ptr viewportsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->addCoordinateSystem (1.0);

  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

  return (viewer);
}


unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

pcl::visualization::PCLVisualizer::Ptr interactionCustomizationVis ()
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
  viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());

  return (viewer);
}



const String keys =
    "{help h usage ? |                  | print this message                                                }"
    "{@left          |../data/aloeL.jpg | left view of the stereopair                                       }"
    "{@right         |../data/aloeR.jpg | right view of the stereopair                                      }"
    "{GT             |../data/aloeGT.png| optional ground-truth disparity (MPI-Sintel or Middlebury format) }"
    "{dst_path       |None              | optional path to save the resulting filtered disparity map        }"
    "{dst_raw_path   |None              | optional path to save raw disparity map before filtering          }"
    "{algorithm      |sgbm              | stereo matching method (bm or sgbm)                               }"
    "{filter         |wls_conf          | used post-filtering (wls_conf or wls_no_conf or fbs_conf)         }"
    "{no-display     |                  | don't display results                                             }"
    "{no-downscale   |                  | force stereo matching on full-sized views to improve quality      }"
    "{dst_conf_path  |None              | optional path to save the confidence map used in filtering        }"
    "{vis_mult       |1.0               | coefficient used to scale disparity map visualizations            }"
    "{max_disparity  |96                | parameter of stereo matching                                      }"
    "{window_size    |-1                | parameter of stereo matching                                      }"
    "{wls_lambda     |8000.0            | parameter of wls post-filtering                                   }"
    "{wls_sigma      |1.5               | parameter of wls post-filtering                                   }"
    "{fbs_spatial    |16.0              | parameter of fbs post-filtering                                   }"
    "{fbs_luma       |8.0               | parameter of fbs post-filtering                                   }"
    "{fbs_chroma     |8.0               | parameter of fbs post-filtering                                   }"
    "{fbs_lambda     |128.0             | parameter of fbs post-filtering                                   }"
    ;



// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{

    CommandLineParser parser(argc,argv,keys);
    parser.about("Disparity Filtering Demo");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }

    
    Mat K= (Mat1d(3, 3) << FX, 0, CX, 0, FY, CY, 0, 0, 1);
    Mat LT= (Mat1d(3, 4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);
    Mat RT= (Mat1d(3, 4) << 0, 0, 0, 10.f, 0, 0, 0, 0, 0, 0, 0, 1);

    String left_im = String("/home/uhnder/Downloads/NTSD-complete-v1.0.1/NewTsukubaStereoDataset/illumination/fluorescent/L_00001.png");
    String right_im = String("/home/uhnder/Downloads/NTSD-complete-v1.0.1/NewTsukubaStereoDataset/illumination/fluorescent/R_00001.png");
    String GT_path = parser.get<String>("GT");

    String dst_path = parser.get<String>("dst_path");
    String dst_raw_path = parser.get<String>("dst_raw_path");
    String dst_conf_path = parser.get<String>("dst_conf_path");
    String algo = String("sgbm");
    String filter = parser.get<String>("filter");
    bool no_display = parser.has("no-display");
    bool no_downscale = true;
    int max_disp = parser.get<int>("max_disparity");
    double lambda = parser.get<double>("wls_lambda");
    double sigma  = parser.get<double>("wls_sigma");
    double fbs_spatial = parser.get<double>("fbs_spatial");
    double fbs_luma = parser.get<double>("fbs_luma");
    double fbs_chroma = parser.get<double>("fbs_chroma");
    double fbs_lambda = parser.get<double>("fbs_lambda");
    double vis_mult = parser.get<double>("vis_mult");

    int wsize;
    if(parser.get<int>("window_size")>=0) //user provided window_size value
        wsize = parser.get<int>("window_size");
    else
    {
        if(algo=="sgbm")
            wsize = 5; //default window size for SGBM
        else if(!no_downscale && algo=="bm" && filter=="wls_conf")
            wsize = 7; //default window size for BM on downscaled views (downscaling is performed only for wls_conf)
        else
            wsize = 15; //default window size for BM on full-sized views
    }

    if (!parser.check())
    {
        parser.printErrors();
        return -1;
    }

    //! [load_views]
    Mat left  = imread(left_im , 0);
    if ( left.empty() )
    {
        cout<<"Cannot read image file: "<<left_im;
        return -1;
    }

    Mat right = imread(right_im, 1);
    if ( right.empty() )
    {
        cout<<"Cannot read image file: "<<right_im;
        return -1;
    }
    //! [load_views]
    
    Ptr<SURF> surf = SURF::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    Mat Desc1, Desc2;
    vector<KeyPoint> kp1, kp2;
    vector<vector<DMatch>> knn_matches;
    surf->detectAndCompute(left, noArray(), kp1, Desc1);
    surf->detectAndCompute(right, noArray(), kp2, Desc1);
    matcher->knnMatch(Desc1, Desc2, knn_matches, 2);
    vector<DMatch> good_matches;
        // //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.5f;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    std::vector<Point2f> leftP;
    std::vector<Point2f> rightP;
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        Point2f newLeftPt = kpLeft[ good_matches[i].queryIdx ].pt;
        Point2f newRightPt = kpLeft[ good_matches[i].queryIdx ].pt;
        if(newLeftPt.y != newRightPt.y)
        {
            cout<<"y axis doesnt match! \n";
        }

        leftP.push_back( newLeftPt );
        rightP.push_back( newRightPt );
    }

    Mat PtsHomogenised;
    vector<Point3d> Curr3dPoints;
    triangulatePoints(K * LT, K * RT, leftP, rightP, PtsHomogenised);
    for (int i = 0; i < PtsHomogenised.cols; i++) {
        Mat x = PtsHomogenised.col(i);
        x /= x.at<double>(3, 0);
        Point3d p(
            x.at<double>(0, 0),
            x.at<double>(1, 0),
            x.at<double>(2, 0)
        );
        Curr3dPoints.push_back(p);
    }
 
   for(auto i: Curr3dPoints )
    {
      pcl::PointXYZ basic_point;
      basic_pont.x = i.x;
      basic_pont.y = i.x;
      basic_pont.z = i.x;
      basic_cloud_ptr->points.push_back(basic_point);
    }
    basic_cloud_ptr->width = basic_cloud_ptr->size();
    basic_cloud_ptr->height = 1;

  pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(basic_cloud_ptr);


  //--------------------
  // -----Main loop-----
  //--------------------
  while (true)
  {
    if(waitKey(2)== 27)
        break;
    viewer->spinOnce (100);

  }




    


}