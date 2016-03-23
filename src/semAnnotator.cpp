// STD
#include <sys/stat.h>

//UIMA
#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
// ROS
#include <ros/ros.h>
#include <ros/package.h>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/io/MongoDBBridge.h>
#include <rs/utils/time.h>
#include <rs/utils/exception.h>
#include <rs/scene_cas.h>
using namespace uima;


class semAnnotator : public Annotator
{
private:
  float test_param;

  struct SemanticMapItem
  {
    std::string name, type;
    double width, height, depth;
    tf::Transform transform;
  };
  std::vector<SemanticMapItem> semanticMapItems;
 

  void readSemanticMap(const std::string &file)
  {
    const std::string &mapFile = file;
    std::cout << mapFile << std::endl;
    if(mapFile.empty())
    {
      throw_exception_message("Semantic map file not found: " + file);
    }
   cv::FileStorage fs(mapFile, cv::FileStorage::READ);
    std::vector<std::string> names;
    fs["names"] >> names;
    semanticMapItems.resize(names.size());
    for(size_t i = 0; i < names.size(); ++i)
    {
      SemanticMapItem &item = semanticMapItems[i];
      cv::FileNode entry = fs[names[i]];

      item.name = names[i];
      entry["type"] >> item.type;
      entry["width"] >> item.height;
      entry["height"] >> item.depth;
      entry["depth"] >> item.width;

      cv::Mat m;
      entry["transform"] >> m;

      // move frame from center to corner
      /*cv::Mat r = m(cv::Rect(0, 0, 3, 3)).inv();
      cv::Mat t(3, 1, CV_64F);
      t.at<double>(0) = -item.depth / 2;
      t.at<double>(1) = -item.width / 2;
      t.at<double>(2) = -item.height / 2;
      t = r * t;
      m.at<double>(0, 3) += t.at<double>(0);
      m.at<double>(1, 3) += t.at<double>(1);
      m.at<double>(2, 3) += t.at<double>(2);*/

      tf::Matrix3x3 rot;
      tf::Vector3 trans;
      rot.setValue(m.at<double>(0, 0), m.at<double>(0, 1), m.at<double>(0, 2), m.at<double>(1, 0), m.at<double>(1, 1), m.at<double>(1, 2), m.at<double>(2, 0), m.at<double>(2, 1), m.at<double>(2, 2));
      trans.setValue(m.at<double>(0, 3), m.at<double>(1, 3), m.at<double>(2, 3));
      item.transform = tf::Transform(rot, trans);
    }
  }

public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
     const std::string &configFile = "/home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/config/semantic_map.yaml";
    boost::property_tree::ptree pt;
    try
    {
      //   boost::property_tree::ini_parser::read_ini(configFile, pt);
    //boost::optional<std::string> semanticMapFile = pt.get_optional<std::string>("tf.semanticMap")
    if(!configFile.empty())
      {
        readSemanticMap(configFile);//semanticMapFile.get());
      }

    }
    catch(boost::property_tree::ini_parser::ini_parser_error &e)
      {
	throw_exception_message("Error opening config file: " + configFile);
      }
    ctx.extractValue("test_param", test_param);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    //semanticMap.clear();
    std::vector<rs::SemanticMapObject> semanticMap;

    semanticMap.reserve(semanticMapItems.size());
    for(size_t i = 0; i < semanticMapItems.size(); ++i)
    {
      SemanticMapItem &item = semanticMapItems[i];
      rs::SemanticMapObject obj = rs::create<rs::SemanticMapObject>(tcas);
      obj.name(item.name);
      obj.typeName(item.type);
      obj.width(item.width);
      obj.height(item.height);
      obj.depth(item.depth);
      obj.transform(rs::conversion::to(tcas, item.transform));
      semanticMap.push_back(obj);
    }
    cas.set(VIEW_SEMANTIC_MAP, semanticMap);

    /*
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
      outInfo("Test param =  " << test_param);
      cas.get(VIEW_CLOUD,*cloud_ptr);

      outInfo("Cloud size: " << cloud_ptr->points.size());
      outInfo("took: " << clock.getTime() << " ms.");
    */
    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(semAnnotator)
