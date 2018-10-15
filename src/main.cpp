/*********************************
           HEADERS
**********************************/
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <iostream>
#include <fstream>
#include <string>

void printUsage (const char* progName){
  std::cout << "\nUsage: " << progName << " <file.pcd> or <file.ply>"  << std::endl <<
               "[q] to exit" << std::endl;
}


int main(int argc, char **argv){

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PolygonMesh cl;
  std::vector<int> filenames;
  bool file_is_pcd = false;
  bool file_is_ply = false;
  bool file_is_txt = false;
  bool file_is_xyz = false;


  if(argc < 2 or argc > 2){
      printUsage (argv[0]);
      return -1;
  }

  pcl::console::TicToc tt;
  pcl::console::print_highlight ("Loading ");

  filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
  if(filenames.size()<=0){
      filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
      if(filenames.size()<=0){
          filenames = pcl::console::parse_file_extension_argument(argc, argv, ".txt");
          if(filenames.size()<=0){
              filenames = pcl::console::parse_file_extension_argument(argc, argv, ".xyz");
              if(filenames.size()<=0){
                  printUsage (argv[0]);
                  return -1;
              }else if(filenames.size() == 1){
                  file_is_xyz = true;
              }
          }else if(filenames.size() == 1){
             file_is_txt = true;
        }
    }else if(filenames.size() == 1){
          file_is_pcd = true;
    }
  }
  else if(filenames.size() == 1){
      file_is_ply = true;
  }else{
      printUsage (argv[0]);
      return -1;
  }

  if(file_is_pcd){
      if(pcl::io::loadPCDFile(argv[filenames[0]], *cloud) < 0){
          std::cout << "Error loading point cloud " << argv[filenames[0]]  << "\n";
          printUsage (argv[0]);
          return -1;
      }
      pcl::console::print_info("\nFound pcd file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->size ());
      pcl::console::print_info (" points]\n");
    }else if(file_is_ply){
      pcl::io::loadPLYFile(argv[filenames[0]],*cloud);
      if(cloud->points.size()<=0 or cloud->points.at(0).x <=0 and cloud->points.at(0).y <=0 and cloud->points.at(0).z <=0){
          pcl::console::print_warn("\nloadPLYFile could not read the cloud, attempting to loadPolygonFile...\n");
          pcl::io::loadPolygonFile(argv[filenames[0]], cl);
          pcl::fromPCLPointCloud2(cl.cloud, *cloud);
          if(cloud->points.size()<=0 or cloud->points.at(0).x <=0 and cloud->points.at(0).y <=0 and cloud->points.at(0).z <=0){
              pcl::console::print_warn("\nloadPolygonFile could not read the cloud, attempting to PLYReader...\n");
              pcl::PLYReader plyRead;
              plyRead.read(argv[filenames[0]],*cloud);
              if(cloud->points.size()<=0 or cloud->points.at(0).x <=0 and cloud->points.at(0).y <=0 and cloud->points.at(0).z <=0){
                  pcl::console::print_error("\nError. ply file is not compatible.\n");
                  return -1;
              }
          }
       }

      pcl::console::print_info("\nFound ply file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->points.size ());
      pcl::console::print_info (" points]\n");
    }else if(file_is_txt or file_is_xyz){
      std::ifstream file(argv[filenames[0]]);
      if(!file.is_open()){
          std::cout << "Error: Could not find "<< argv[filenames[0]] << std::endl;
          return -1;
      }
      
      std::cout << "file opened." << std::endl;
      double x_,y_,z_;
      uint8_t r_,g_,b_; 

      while(file >> x_ >> y_ >> z_ /*>> r_ >> g_ >> b_*/){
          pcl::PointXYZRGB pt;
          pt.x = x_;
          pt.y = y_;
          pt.z= z_;                  
              
          cloud->points.push_back(pt);
          //std::cout << "pointXYZRGB:" <<  pt << std::endl;
      }      
     
      pcl::console::print_info("\nFound txt file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->points.size ());
      pcl::console::print_info (" points]\n");
    }

  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;
  cloud->is_dense = true;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL VISUALIZER"));

  viewer->setPosition(0,0);
  viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0); // Setting background to a dark grey

  viewer->addCoordinateSystem();
  pcl::PointXYZ p1, p2, p3;

  p1.getArray3fMap() << 1, 0, 0;
  p2.getArray3fMap() << 0, 1, 0;
  p3.getArray3fMap() << 0,0.1,1;

  viewer->addText3D("x", p1, 0.2, 1, 0, 0, "x_");
  viewer->addText3D("y", p2, 0.2, 0, 1, 0, "y_");
  viewer->addText3D ("z", p3, 0.2, 0, 0, 1, "z_");

  if(cloud->points[0].r <= 0 and cloud->points[0].g <= 0 and cloud->points[0].b<= 0 ){
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(cloud,255,255,0);
      viewer->removeAllPointClouds(0);
      viewer->addPointCloud(cloud,color_handler,"POINTCLOUD");
  }else{
      viewer->addPointCloud(cloud,"POINTCLOUD");
  }
  
  // viewer->addPointCloud(cloud,"POINTCLOUD");
  viewer->initCameraParameters();
  viewer->resetCamera();

  pcl::console::print_info ("\npress [q] to exit!\n");

  while(!viewer->wasStopped()){
      viewer->spin();
  }


  return 0;
}

