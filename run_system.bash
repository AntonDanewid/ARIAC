sudo rm -r build

cd src
rosdep install --from-paths . --ignore-src --rosdistro kinetic
cd ..
catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin_make

source $PWD/devel/setup.bash

rosrun curlnanton ariac_example_node.py
