Repository for EDAN70 project based on ARIAC competition

Participants: 
Carl Dahl <carl.dahl.181@student.lu.se> and Anton Danewid <anton.danewid@gmail.com>

Supervisor: Jacek Malec <jacek.malec@cs.lth.se>

# Running instructions
 
To run this system, first follow the guide on the ARIAC 2018 website to install the required dependencies for ROS and ARIAC. 

Next, install moveit by following the guide provided on the moveit webiste. 

After pulling the project, you need to run the command cmake build to build all the required packages for the project. This is only required once. 

After everything has been built, run the followin commands in the following order in separate terminal windows. 
* catkin_make install
* ./run_ariac
* ./run_moveit
* ./run_system


