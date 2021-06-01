echo "Building ROS nodes"

cd Examples/ROS/VIEO_SLAM
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
