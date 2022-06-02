#pragma once
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <map>
#include <string>

namespace CloudParserLibrary {

class InterfaceParser {
 public:
  virtual void load_cloudfile(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {}
};

}  // namespace CloudParserLibrary