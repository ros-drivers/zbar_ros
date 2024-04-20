# Zbar ROS

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=humble)](../../actions/workflows/build_and_test_humble.yaml?query=branch:humble)
[![Build and Test (iron)](../../actions/workflows/build_and_test_iron.yaml/badge.svg?branch=iron)](../../actions/workflows/build_and_test_iron.yaml?query=branch:iron)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=ros2)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:ros2)

Basic ROS2 wrapper for the zbar (http://zbar.sourceforge.net/) barcode reader library. Reads image stream from `image` topic, and outputs detected barcodes to `barcode` topic. Works with 1D and 2D barcodes.

![Usage](images/usage.gif)

## Installation

### Binary Installation

To perform a binary installation, source your ROS 2 installation, then simply run:

```sh
sudo apt install ros-${ROS_DISTRO}-zbar-ros
```

### Source Installation (Alternative)

Alternatively, you can build from source.

Make sure you are in a ROS 2 workspace.
Clone this repository into your workspace's ``src`` directory by running:

```sh
git clone git@github.com:ros-drivers/zbar_ros.git src/zbar_ros
```

Install the dependencies using rosdep:

```sh
rosdep install --from-paths src --ignore-src -r -y
```

Build the package:

```sh
colcon build
```

## Usage

You have to source your workspace, then run the node. In your workspace, run:

```
source install/local_setup.bash
ros2 run zbar_ros barcode_reader
```

### Topics

Subscriptions:
* `image` (`sensor_msgs/msg/Image`)

Publisher:
* `symbol` (`zbar_ros_interfaces/msg/Symbol`)
* `barcode` (`std_msgs/msg/String`) - **DEPRECATED**


## Debugging the barcode_reader node

To debug whether the node is
* receiving msgs on `image`
* detecting your 1d or 2d QT code
* publishing the result on `barcode`

run with debug logging enabled as below:

```
ros2 run zbar_ros barcode_reader --ros-args --log-level DEBUG
```
