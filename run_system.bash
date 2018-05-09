cd src
rosdep install --from-paths . --ignore-src --rosdistro kinetic
source /opt/ros/kinetic/setup.bash
cd ..
catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build curlnanton
source /opt/ros/kinetic/setup.bash
source devel/setup.bash
rosrun curlnanton ariac_example_node.py
#rosrun curlnanton planner_node.py