cd src
rosdep install --from-paths . --ignore-src --rosdistro kinetic
source /opt/ros/kinetic/setup.bash
cd ..
catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release
source /opt/ros/kinetic/setup.bash
source $PWD/devel/setup.bash
catkin build
rosrun curlnanton ariac_planner_node.py
