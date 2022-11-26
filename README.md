
```
# Raspi 4
https://www.raspberrypi.com/products/raspberry-pi-4-model-b/

# 512 Gb U3 A2 SDXC V30 Micro SD card
https://www.amazon.com/SanDisk-Extreme-microSDXC-Memory-Adapter/dp/B07P7M6K35

# RasPi High Quality camera
https://www.raspberrypi.com/products/raspberry-pi-high-quality-camera/

# Install RasPi OS Bullseye
https://www.raspberrypi.com/news/raspberry-pi-os-debian-bullseye/
Raspberry Pi OS (other) -> Raspberry Pi OS (64-bit)

# Use Raspberry Pi Imager tool
https://www.raspberrypi.com/news/raspberry-pi-imager-imaging-utility/
https://www.raspberrypi.com/software/

# Or manually install
https://www.raspberrypi.com/software/operating-systems/#raspberry-pi-os-64-bit

# Edit boot config to support the High Quality camera. 
/boot/config.txt:
[all]
camera_auto_detect=0
dtoverlay=imx477,media-controller=0
gpu_mem=256
dtoverlay=vc4-kms-v3d


# Re-enable the legacy config with raspi-config
https://www.youtube.com/watch?v=E7KPSc_Xr24

# Install ROS2 per instructions at https://docs.ros.org/en/galactic/Installation/Maintaining-a-Source-Checkout.html (broken out below)
export ROS_DISTRO=galactic
sudo pip install -U vcstool
mkdir ~/ros2_galactic
cd ~/ros2_galactic
wget https://raw.githubusercontent.com/ros2/ros2/galactic/ros2.repos
mkdir src
vcs custom --args remote update
vcs import src < ros2.repos
vcs pull src
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install python3-colcon-common-extensions
sudo apt-get install python3-rosdep2 
sudo rosdep init
rosdep update
cd src/
git clone https://github.com/iRobotEducation/irobot_create_msgs.git -b galactic
cd ..
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile (set CONF_MAXSWAP=8192, CONF_SWAPSIZE=, and CONF_SWAPFACTOR=4, 
sudo dphys-swapfile setup
sudo dphys-swapfile swapon

rosdep install -r --from-paths . --ignore-src --rosdistro galactic -y
colcon build --symlink-install --parallel-workers 20 --event-handlers console_direct+ 

. install/local_setup.bash
echo source /home/pi/ros2_galactic/install/local_setup.bash >> ~/.bashrc

# update with dynamixal support:
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b galactic-devel
colcon build --symlink-install --parallel-workers 20 --packages-skip-build-finished --event-handlers console_direct+ 


# update with rplidar support 
cd src
git clone https://github.com/Slamtec/rplidar_ros.git -b ros2
pico +12 rplidar_ros/CMakeLists.txt (add -Wno-narrowing)
colcon build --symlink-install --parallel-workers 20 --packages-skip-build-finished --event-handlers console_direct+ 


# start getting fancy with nav2
git clone https://github.com/SeanReg/nav2_wavefront_frontier_exploration.git nav2_wfd
pico /home/pi/ros2_galactic/src/nav2_wfd/package.xml (adjust line road  <maintainer email="stop@warning.com">Sean Regan</maintainer> )

git clone https://github.com/ros-planning/navigation2.git nav2 -b galactic
git clone https://github.com/SteveMacenski/slam_toolbox.git -b galactic
git clone https://github.com/ros/bond_core.git -b galactic 
git clone https://github.com/ompl/ompl.git
git clone https://github.com/ros-perception/vision_opencv.git -b galactic
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git -b v3.8
#git clone https://github.com/ros2/common_interfaces.git -b galactic 
git clone https://github.com/robo-friends/m-explore-ros2.git
cd m-explore-ros2
git config --global user.email "you@example.com"
git config --global user.name "Your Name"
git pull origin pull/26/head
cd ..
rosdep install -r --from-paths . --ignore-src --rosdistro galactic -y
pico /home/pi/ros2_galactic/build/pluginlib/pluginlib_enable_plugin_testing/install/test_pluginlib__test_pluginlib/share/test_pluginlib/package.xml (add <maintainer email="stop@warning.com">Sean Regan</maintainer> and <license>who cares License</license>)

# build without Gazebo support
colcon build --symlink-install --parallel-workers 20 --packages-skip-build-finished --event-handlers console_direct+ --packages-ignore nav2_system_tests

# Add Gazebo dependencies 
sudo apt-get install libfreeimage-dev libfreeimageplus-dev
sudo apt-get install libprotobuf-dev libprotobuf-c-dev
sudo apt-get install libtar-dev
sudo apt-get install libsdformat6-dev 
sudo apt-get install libopenal-dev
sudo apt-get install libsdformat6-dev 
sudo apt-get install libgraphviz-dev 
sudo apt-get install liboctovis-dev
sudo apt-get install xsltproc
sudo apt-get install libhdf5-dev 
sudo apt-get install libsimbody-dev 
sudo apt-get install libqwt-qt5-dev
sudo apt-get install libdart-all-dev 
sudo apt-get install libignition-transport4-dev libignition-math4-dev libignition-msgs-dev
sudo apt-get install libignition-fuel-tools1-dev 
sudo apt-get install ruby-dev
sudo apt-get install libdart-external-ikfast-dev
sudo apt-get install openjdk-11-jdk-headless

cd src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b galactic
git clone https://github.com/gazebosim/gazebo-classic.git -b gazebo9
git clone https://github.com/OGRECave/ogre -b v1.9.1 
git clone https://github.com/gazebosim/sdformat -b sdf6
git clone https://github.com/gazebosim/gz-tools.git -b ign-tools1
git clone https://github.com/gazebosim/gz-math.git -b ign-math4
git clone https://github.com/gazebosim/gz-cmake.git -b ign-cmake0
git clone https://github.com/ignitionrobotics/ign-transport -b ign-transport4
git clone https://github.com/ignitionrobotics/ign-msgs -b ign-msgs1

cd ..
# build without all the testing & don't stop, but include Gazebo! 
colcon build --symlink-install --parallel-workers 20 --packages-skip-build-finished --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF --continue-on-error --cmake-args -DCMAKE_CXX_STANDARD=17 --packages-up-to nav2_system_tests

# add Intel RealSense D4xx
sudo apt-get install automake libtool vim cmake libusb-1.0-0-dev libx11-dev xorg-dev libglu1-mesa-dev
cd ~
git clone https://github.com/IntelRealSense/librealsense.git (-b v1.12.1 for legacy camera)
cd librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ 
sudo su
udevadm control --reload-rules && udevadm trigger
exit
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
sudo apt-get install python3-protobuf 
mkdir  build  && cd build
make -j1
sudo make install
echo export PYTHONPATH=$PYTHONPATH:/usr/local/lib > ~/.bashrc

rs-enumerate-devices 
cmake .. -DBUILD_PYTHON_BINDINGS=bool:true -DPYTHON_EXECUTABLE=$(which python3)
make -j4
sudo make install
cd ~/ros2_galactic/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
git clone https://github.com/ros/diagnostics.git -b galactic

cd ..
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
colcon build --symlink-install --parallel-workers 20 --packages-skip-build-finished --event-handlers console_direct+ --packages-up-to  realsense2_camera

# Lauch realsense cam
realsense-viewer
ros2 launch realsense2_camera rs_launch.py

# Launch RPLidar s1
sudo chmod 777 /dev/ttyUSB0 
ros2 launch rplidar_ros2 rplidar_s1_launch.py (without rviz)
ros2 launch rplidar_ros2 view_rplidar_s1_launch.py (with rviz)


#############################################
Work in progress - alternative instead
#############################################
wget https://raw.githubusercontent.com/MAVProxyUser/RasPi4_ROS2_realsense_rplidar/main/vcs_repos.txt
cd ~/ros2_galactic/
sudo pip install -U vcstool
vcs import src < vcs_repos.txt
ros2 pkg create --build-type ament_cmake janitor
edit janitor/package.xml
(
add:
  <depend>libfreeimage-dev</depend>
  <depend>libfreeimageplus-dev</depend>
  <depend>libprotobuf-dev</depend>
  <depend>libprotobuf-c-dev</depend>
  <depend>libtar-dev</depend>
  <depend>libsdformat6-dev</depend>
  <depend>libopenal-dev</depend>
  <depend>libgraphviz-dev</depend>
  <depend>liboctovis-dev</depend>
  <depend>xsltproc</depend>
  <depend>libhdf5-dev</depend>
  <depend>libsimbody-dev</depend>
  <depend>libqwt-qt5-dev</depend>
  <depend>libdart-all-dev</depend>
  <depend>libignition-transport4-dev</depend>
  <depend>libignition-math4-dev</depend>
  <depend>libignition-msgs-dev</depend>
  <depend>libignition-fuel-tools1-dev</depend>
  <depend>ruby-dev</depend>
  <depend>openjdk-11-jdk-headless</depend>
  <depend>python3-colcon-common-extensions</depend>
  <depend>python3-rosdep2</depend>
  <depend>python3-pyopengl</depend>
)
rosdep install -r --from-paths janitor/ --ignore-src -y

```
