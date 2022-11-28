![logo](docs/resources/ARU_logo_rectangle.png)
# DV XPLORER ROS2 DRIVER

This repo aims provides a generic [ROS2](https://docs.ros.org/en/foxy/index.html) driver for the [Dynamic Vision iniVation DVXplorer sensor](https://inivation.com/solution/dvp/). The repo was adapted from [Robotics and Perception Group's](https://github.com/uzh-rpg/rpg_dvs_ros/tree/master/dvxplorer_ros_driver) ROS1 version. And so credit is given to this version for much of the code and convention in this driver. 

<hr/> 
## Prerequisites
<hr/>

### ROS2
- Tested on [ROS2 foxy](https://docs.ros.org/en/foxy/Installation.html)
<hr/>

### DV Repositories
Installed and tested on Ubuntu 20.04 LTS.

Add required repositories for Ubuntu Linux as per the [iniVation documentation](https://inivation.gitlab.io/dv/dv-docs/docs/getting-started.html#ubuntu-linux)

```bash
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt-get update
sudo apt-get install dv-runtime dv-runtime-dev
```

### Libcaer

```bash
sudo apt-get install libcaer-dev
```

## Running the ros package

clone this repo to your ros2 workspace source directory and build the packages:
```bash
git clone <somelink>
colcon build --packages-select dvxplorer_interfaces dvxplorer_ros2 dvs_renderer  
```
Modify camera parameters as desired in the [config](dvxplorer_ros2/config/dvxplorer_config.yaml) and [launch](dvxplorer_ros2/launch/dvxplorer_capture_mono.launch.xml) files, and run the node using the provided launch file or a custom. 

## DVS RENDERER
To view the published images, run the [dvs_renderer](dvs_renderer) node using the provided [launch](dvs_renderer/launch/dvxploerer_mono.launch.xml) file. 