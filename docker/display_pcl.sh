# Allow X server connection
xhost +local:root
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=/home/danieltc/Downloads/pcl_visualizer/build:/tmp \
    pcl-visualizer:1.0 /tmp/$1
# Disallow X server connection
xhost -local:root