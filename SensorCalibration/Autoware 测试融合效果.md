# Autoware 测试融合效果

启动 ros：

```shell
roscore
```

回放 bag 启动立刻暂停，Autoware 接收的点云主题为 `/points_raw`：

```shell
rosbag play --pause zed_lidar_calibration-2020-05-30-16-24-45.bag /rslidar_points:=/points_raw
```

启动 Autoware：

```shell
roslaunch runtime_manager runtime_manager.launch
```

点击 calibration_publisher 加载外参标定文件：





点击 points_image 设置相机主题：





点击底部的 Rviz，AddNewPlanel->ImageViewPlugin，选择相机话题 left，选择融合话题 points_image，开始回放数据包，即可看到融合效果：