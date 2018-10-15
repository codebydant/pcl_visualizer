# pcl_visualizer
Visualizer for 3D point cloud using PCL Library 1.8.1 C++

----------------------

This program display a PCL viewer for input data with next extension:

* PCD 
* PLY 
* TXT 
* XYZ 

## Example

<img src="./example/example.png" align="center" height="500" width="640"><br>

## Compile
* Set "YOUR" PCL Build DIR in CMakeList.txt e.g: /opt/pcl-1.8.1/build and saved.
* Create a "build" folder

in the main folder:

	- src 
	- build
Then:

	- cd build  
	- cmake ../src/
  	- make
       
        	 
### Test

	./pcl-visualizer <pcd file> 
  	./pcl-visualizer <ply file> 
  	./pcl-visualizer <txt file> 
  	./pcl-visualizer <xyz file> 
  


