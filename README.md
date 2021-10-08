# README #

Basic ROS2 wrapper for the zbar (http://zbar.sourceforge.net/) barcode reader library. Reads image stream from `image` topic, and outputs detected barcodes to `barcode` topic. Works with 1D and 2D barcodes.


## Running barcode_reader node

All you have to do is run:

```
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
