/*********************************
           HEADERS
**********************************/
#include <pcl/visualization/pcl_visualizer.h>

#include <modern/parser.hpp>

void printUsage(const char *progName) {
  std::cout << "\nUse: " << progName << " <file>" << std::endl
            << "support: .pcd .ply .txt .xyz" << std::endl
            << "[q] to exit" << std::endl;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    printUsage(argv[0]);
    std::exit(-1);
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  CloudParserLibrary::ParserCloudFile parser;
  parser.load_cloudfile("Tree1.pcd", cloud);

  cloud->width = (int)cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  // Disable vtk render warning
  vtkObject::GlobalWarningDisplayOff();
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new
  // pcl::visualization::PCLVisualizer ("PCL VISUALIZER"));
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL VISUALIZER"));

  viewer->setPosition(0, 0);
  viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0);

  viewer->setShowFPS(false);

  viewer->addCoordinateSystem();
  pcl::PointXYZ p1, p2, p3;

  p1.getArray3fMap() << 1, 0, 0;
  p2.getArray3fMap() << 0, 1, 0;
  p3.getArray3fMap() << 0, 0.1, 1;

  viewer->addText3D("x", p1, 0.2, 1, 0, 0, "x_");
  viewer->addText3D("y", p2, 0.2, 0, 1, 0, "y_");
  viewer->addText3D("z", p3, 0.2, 0, 0, 1, "z_");

  std::string str = "Points: ";
  std::stringstream ss;
  ss << cloud->points.size();
  str += ss.str();

  int xpos = 1.0;
  int ypos = 1.0;
  int fontSize = 13;

  double r = 1.0;
  double g = 1.0;
  double b = 1.0;

  viewer->addText(str, xpos, ypos, fontSize, r, g, b, "text1");

  if (cloud->points[0].r <= 0 and cloud->points[0].g <= 0 and cloud->points[0].b <= 0) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(cloud, 255, 255, 0);
    viewer->removeAllPointClouds(0);
    viewer->addPointCloud(cloud, color_handler, "POINTCLOUD");
  } else {
    viewer->addPointCloud(cloud, "POINTCLOUD");
  }

  viewer->initCameraParameters();
  viewer->resetCamera();

  pcl::console::print_info("\npress [q] to exit!\n");

  viewer->spin();
  viewer->close();

  return 0;
}
