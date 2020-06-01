内参标定比较简单，基本都是自动执行，先 source Autoware 环境以使用标定工具：

```shell
cd autoware-1.10.0/ros/

source devel/setup.zsh
```

启动 roscore：

```shell
roscore
```

启动标定工具 `autoware_camera_lidar_calibrator`：

```shell
rosrun autoware_camera_lidar_calibrator cameracalibrator.py --square 0.025  --size 11x8  left:=/camera/left/image_raw right:=/camera/right/image_raw
```

参数如下：

- `--square`：标定板单元格的边长（m），我的标定板是 2.5cm，也就是 0.025m
- `--size`：标定板长x宽的格子数减一，我的标定板是 12x9，所以填 11x8
- `right`：相机左话题
- `left`：相机右话题

启动后就是一个黑窗口：

![zed_display](C:\Users\dlonn\Desktop\AI-Notes\SensorCalibration\标定博客图片\zed_display.png)

然后开始回放内参标定 Bag，默认暂停启动，按空格继续：

```shell
rosbag play --pause zed_calibration.bag
```

标定过程如下，标定工具会根据棋盘格位置自动检测角点：

![zed_left_right](C:\Users\dlonn\Desktop\AI-Notes\SensorCalibration\标定博客图片\zed_left_right.png)

当右上角的 X、Y、Size、Skew 变为绿色时，标定按钮「CALIBRATE」可用，点击即可计算内参矩阵：

![zed_calibr_left_right](C:\Users\dlonn\Desktop\AI-Notes\SensorCalibration\标定博客图片\zed_calibr_left_right.png)

结果在 Shell 中打印出来，点击「SAVE」可保存到 home 目录下：