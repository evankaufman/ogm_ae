Experiment and Simulation of occupancy grid mapping package
To run the simulation follow the steps below

1. Run launch file for starting up stage simulator: roslaunch ogm_ae start_stage.launch
2. Run launch file for running mapping and navigation: roslaunch ogm_ae start_exploration.launch
3. Run node for getting /goal to move the robot: rosrun ogm_ae: cmdpose_tests.py
  * (This node might be better to be included in the launch file)

The map of the environment can be selected in the start_stage.launch file. However, the robot setting and environmental change can be made within ogm_ae/stage-worlds/. The image file for the stage, sensor range and max measurements can be also modified in the pioneer3dx-hokuyo.world

Other parameters for the robot mapping and navigation need to be modified in the start_exploration.launch and c++ source code such as run_robot.cpp and plot_map.cpp. Because current simulation replaces the robot to new desired position pioneer_traj_control.cpp is no longer used for the simulation. It is still used for the actual experiment at NRL lab.


Configuration specific:
* required packages:
* stage_ros need to be checked out for the branch for replacing the robot (add_pose_sub) https://github.com/ros-simulation/stage_ros
* may require gazebo_ros_pkg kinetic-devel branch installation from gitrepo [ros-wiki](http://wiki.ros.org/gazebo_ros)

The following packages run on ROS Kinetic (Ubuntu 16.04) using Gazebo 8. The full instructions (command line) are below:

Install ROS Kinetic:
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
IMPORTANT NOTE: the above command may not work if qt5 is installed on your system.
sudo rosdep init (you might have to delete a file if you installed ROS before, e.g., sudo rm /etc/ros/rosdep/sources.list.d/20-default.list)
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential

Several other packages are required (some may already be installed):
sudo apt-get install ros-kinetic-tf2*
sudo apt-get install ros-kinetic-voxel-grid
sudo apt-get install ros-kinetic-hector-*

Without desktop-full ROS install, you might also need:
sudo apt-get install libgtest-dev
sudo apt-get install ros-kinetic-robot-state-publisher
sudo apt-get install ros-kinetic-pcl-*

Note: in some cases, the ROS package may not be updated yet for the distribution (e.g. Hector with ROS Lunar on 7/17/17).

Gazebo: IMPORTANT NOTE! ROS Kinetic uses Gazebo 7. On some machines, the fsaa argument (https://bitbucket.org/osrf/gazebo/issues/1837/vmware-rendering-z-ordering-appears-random#comment-29498034) causes camera depth readings to crash Gazebo. One solution is removing Gazebo 7, downloading Gazebo 8 (http://gazebosim.org/tutorials?tut=install_ubuntu), then reinstalling the ros packages gazebo_ros and gazebo_plugins. Instructions to do this are below:
sudo apt-get purge gazebo7
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
cat /etc/apt/sources.list.d/gazebo-stable.list
  -> CHECK: this gives deb http://packages.osrfoundation.org/gazebo/ubuntu-stable trusty main
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo8
sudo apt-get install libgazebo8-dev
sudo apt-get install ros-kinetic-gazebo8*

Connecting Master & Slave
Disable any firewalls (e.g. sudo service ufw stop)
On Master, add a .sh file with 
export ROS_IP=master_ip
where master_ip is the master's IP and source it.
On Slave, add a .sh file with
export ROS_IP=slave_ip
export ROS_MASTER_URI=http://master_ip:11311
where slave_ip is the slave's IP and source it.

Hokuyo connection (urg_node):
Apply the follow network settings (manual IPv4):
IP:      192.168.0.15
Netmask: 24
Gateway: 192.168.0.1
After the network configuration, the service might need to restart:
service networking restart
Once installed (apt-get/Github), run (for the Ethernet version)
rosrun urg_node urg_node _ip_address:="192.168.0.10"
and look for the "/scan" message.

Jetson TX2 Cloning without ConnectTech Board: simply follow http://elinux.org/Jetson/TX2_Cloning.

Jetson TX2 Cloning with ConnectTech Board: for either copying the image or flashing the copied image, make sure the board is in recovery mode (turn on with power (and "soc" on development board), hold down "rec" 4 seconds, striking and releasing "rst" in the middle of holding. Navigate to the 64_TX2/Linux_for_Tegra_tx2 directory, then...
Copy (clone) the master image (command from http://elinux.org/Jetson/TX2_Cloning):
sudo ./flash.sh -r -k APP -G system.img jetson-tx2 mmcblk0p1
Important note: this MUST be done on the ConnectTech breakout board if the drivers for that board were installed on the master.
The above will create system.img & system.img.raw. You will need to replace the existing system.img inside the bootloader folder with this new one.
Flash new cloned Jetson (commands combined from above link and ConnectTech driver ReadMe assuming you're using the orbitty board):
sudo ./flash.sh -r orbitty mmcblk0p1
Note: this is successfully tested on the development breakout board and not on the ConnectTech board. Then the Jetson works on both.


GTest
First, install on your machine:
If catkin_make fails (Could NOT find GTest (missing: GTEST_LIBRARY GTEST_MAIN_LIBRARY)), try compiling the gtest library:
cd /usr/src/gtest
sudo cmake .
sudo make
sudo cp libg* /usr/lib/


