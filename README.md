# pcl_visualizer
Visualizer for a 3D point cloud using PCL Library 1.8...1.12.1

----------------------

This program display a PCL viewer for input point cloud data

## Input file structure support
| Format      | Description |
| ----------- | ----------- |
| .pcd      | Point Cloud Data file format       |
| .ply   | Polygon file format        |
| .txt   | Text file format        |
| .xyz      | X Y Z Text file format       |


## Example
![Screenshot from 2022-06-03 08-21-25](https://user-images.githubusercontent.com/35694200/171862553-c287954b-1a58-4005-8831-c035924ee57f.png)
![Screenshot from 2022-06-03 08-26-18](https://user-images.githubusercontent.com/35694200/171863365-5e259039-7980-47fa-a4ef-34ac20e8db6d.png)
![Screenshot from 2022-06-03 08-31-38](https://user-images.githubusercontent.com/35694200/171864244-aac54f80-6bb0-4ec5-ab0c-e19f669163a1.png)

## Help
```
| Help:
-------
          p, P   : switch to a point-based representation
          w, W   : switch to a wireframe-based representation (where available)
          s, S   : switch to a surface-based representation (where available)

          j, J   : take a .PNG snapshot of the current window view
          c, C   : display current camera/window parameters
          f, F   : fly to point mode

          e, E   : exit the interactor
          q, Q   : stop and call VTK's TerminateApp

           +/-   : increment/decrement overall point size
     +/- [+ ALT] : zoom in/out 

          g, G   : display scale grid (on/off)
          u, U   : display lookup table (on/off)

    o, O         : switch between perspective/parallel projection (default = perspective)
    r, R [+ ALT] : reset camera [to viewpoint = {0, 0, 0} -> center_{x, y, z}]
    CTRL + s, S  : save camera parameters
    CTRL + r, R  : restore camera parameters

    ALT + s, S   : turn stereo mode on/off
    ALT + f, F   : switch between maximized window mode and original size

          l, L           : list all available geometric and color handlers for the current actor map
    ALT + 0..9 [+ CTRL]  : switch between different geometric handlers (where available)
          0..9 [+ CTRL]  : switch between different color handlers (where available)

    SHIFT + left click   : select a point (start with -use_point_picking)

          x, X   : toggle rubber band selection mode for left mouse button
```

## Compilation
This program depends on PCL, VTK and OpenGL.

1. In the root folder create a build directoy

```
mkdir build
```
2. Create compilation files
   
```
cd build && cmake ../src
```
3. Compile project
   
```
make
```

Expected output

![Screenshot from 2022-06-03 08-35-43](https://user-images.githubusercontent.com/35694200/171864937-91aa8a3d-8bff-4f08-a3f8-d084a6243b5c.png)
       
        	 
## Test
```
./pcl-visualizer <cloud file> 
```
Expected output

![Screenshot from 2022-06-03 08-39-30](https://user-images.githubusercontent.com/35694200/171865601-c64efd17-a088-4f3f-afda-c62d20d04f93.png)

## Docker
This [image](https://hub.docker.com/r/danieltobon43/pcl-visualizer) is based on Linux Alpine 3.15 and has the following packages installed:

- VTK-9.1.0
- PCL-1.12.0
- Eigen-3.7.7
- Flann-1.9.1
- Boost-1.77.0

It's a lightweight [1.27GB] PCL docker image with the visualization module pre-compiled that uses the pcl-visualizer project to display a cloud

**PCL modules:**
```
The following subsystems will be built:
--   common
--   kdtree
--   octree
--   search
--   geometry
--   io
--   visualization
-- The following subsystems will not be built:
--   sample_consensus: Disabled manually.
--   filters: Disabled manually.
--   2d: Disabled manually.
--   features: Disabled manually.
--   ml: Disabled manually.
--   segmentation: Disabled manually.
--   surface: Disabled manually.
--   registration: Disabled manually.
--   keypoints: Disabled manually.
--   tracking: Disabled manually.
--   recognition: Disabled manually.
--   stereo: Disabled manually.
--   apps: Disabled by default
--   benchmarks: Disabled by default
--   outofcore: Disabled manually.
--   examples: Code examples are disabled by default.
--   people: Disabled manually.
--   simulation: Disabled by default.
--   global_tests: Disabled by default
--   tools: Disabled manually.
-- Configuring done
-- Generating done
```

## Download image from Docker hub
```
docker pull danieltobon43/pcl-visualizer:1.0-alpine3.15
```

## Run docker image
1. Create a `visualizer.sh` file with executable permissions.
2. Copy the next content into the `visualizer.sh` file (remember to update PATH/TO/YOUR/PCD/PLY/FOLDER accordingly):
```
# Allow X server connection
xhost +local:root
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=PATH/TO/YOUR/PCD/PLY/FOLDER:/tmp \
    danieltobon43/pcl-visualizer:1.0-alpine3.15 /tmp/$1
# Disallow X server connection
xhost -local:root
```
3. Run the command:
```
./visualizer.sh tmp/YOUR-PCD-PLY-FILENAME
```