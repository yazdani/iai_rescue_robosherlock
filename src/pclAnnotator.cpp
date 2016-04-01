#include <uima/api.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <rs/types/all_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/segmentation/progressive_morphological_filter.h>

//project
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/utils/common.h>
#include <opencv2/opencv.hpp>

using namespace uima;


class pclAnnotator : public  DrawingAnnotator
{
private:
  float test_param;
  cv::Mat color;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
  pcl::PCDReader reader;
  std::vector<cv::Rect> cluster_rois;
  double pointSize;

public:

  pclAnnotator(): DrawingAnnotator(__func__), pointSize(1)
  {
    cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  
    reader.read<pcl::PointXYZ> ("/home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/clouds/complete.pcd", *cloud_ptr);
 }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    pcl::PointIndicesPtr ground (new pcl::PointIndices);
    //cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ( "/home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/clouds/complete.pcd", *cloud_ptr) == -1) //* load the file
      {
	PCL_ERROR ("Couldn't read file complete.pcd \n");
	return (-1);
      }
    //CREATE THE FILtERING OBJECT
    //  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    // pmf.setInputCloud (cloud_ptr);
    // pmf.setMaxWindowSize (20);
    //pmf.setSlope (1.0f);
    //pmf.setInitialDistance (0.5f);
    //pmf.setMaxDistance (3.0f);
    //pmf.extract (ground->indices);

    ctx.extractValue("test_param", test_param);
    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    // cv::Mat disp;
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    cluster_rois.clear();
    outInfo("Test param =  " << test_param);
    cas.get(VIEW_CLOUD,*cloud_ptr);
    cas.get(VIEW_COLOR_IMAGE_HD, color);
    cas.set(VIEW_CLOUD, *cloud_ptr);
    //  drawImageWithLock(disp);
    outInfo("Cloud size: " << cloud_ptr->points.size());
    outInfo("took: " << clock.getTime() << " ms.");
    outInfo("processWithLock done");

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {    
    // disp = imread(argv[1], CV_LOAD_IMAGE_COLOR); 
     disp = cv::Mat::zeros(980, 1640, CV_8UC3);
  }

void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;
   
    outInfo("fillVisualizerWithLock");
    if(firstRun)
    {
      visualizer.addPointCloud(cloud_ptr, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(cloud_ptr, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    outInfo("fillVisualizerWithLock done");
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(pclAnnotator)
