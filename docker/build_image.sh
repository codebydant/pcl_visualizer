docker build -f Dockerfile_pcl --target runtime -t pcl-docker:1.12.1-alpine3.15 .
docker build -f Dockerfile_dev --target dev-debug -t pcl-docker:1.12.1-alpine3.15-dev .
docker build -f Dockerfile_pcl_visualizer --target runtime -t pcl-visualizer:1.0 .