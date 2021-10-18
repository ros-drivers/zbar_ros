# README #

Basic ROS2 wrapper for the zbar (http://zbar.sourceforge.net/) barcode reader library. Reads image stream from `image` topic, and outputs detected barcodes to `barcode` topic. Works with 1D and 2D barcodes.

## Install Dependencies

To install the dependencies, run:

```
rosdep install --from-paths src --ignore-src -r -y
```


If you see `sudo: rosdep: command not found`, [install rosdep](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html#installing-and-initializing-rosdep) first and rerun the command above.

## Build zbar_ros

In your workspace, run:

```
colcon build
```


## Running barcode_reader node

You have to source your workspace, then run the node. In your workspace, run:

```
. install/local_setup.bash
ros2 run zbar_ros barcode_reader
```

### Topics

Subscriptions:
* `image` (`sensor_msgs/msg/Image.msg`)

Publisher:
* `barcode` (`std_msgs/msg/String.msg`)


### (Optional) Debugging the barcode_reader node

To debug whether the node is
* receiving msgs on `image`
* detecting your 1d or 2d QT code
* publishing the result on `barcode`

run with debug logging enabled as below:

```
ros2 run zbar_ros barcode_reader --ros-args --log-level DEBUG
```
