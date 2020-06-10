## 一、安装 Autoware & ZED 内参标定 & 外参标定准备

之前的这篇文章：[Autoware 进行 Robosense-16 线雷达与 ZED 双目相机联合标定！](https://dlonng.com/posts/autoware-calibr-1) 记录了我用 Autoware 标定相机和雷达的过程，虽然用的不是 Calibration ToolKit 工具，但是博客里面的这些章节也适用本次的  Calibration Tool Kit 工具：

- 一、编译安装 Autoware-1.10.0
- 二、标定 ZED 相机内参
- 3.1 联合标定准备

如果你是第一次看这篇 Calibration Tool Kit 联合标定的博客，建议先按照之前的博客安装 Autoware、标定 ZED 内参和做好外参标定的准备（标定板，录制标定包等），最好用上篇博客的方法标定一次。

所以这篇博客我就直接开始介绍使用 Calibration Tool Kit 标定雷达和相机外参的过程！

## 二、Calibration Tool Kit 联合标定雷达和 ZED 相机

### 2.1 启动 Autoware

先启动 Autoware-1.10.0，启动过程中可能需要输入 root 密码：

```shell
# 1. 进入 autoware 的 ros 目录下
cd autoware-1.10.0/ros

# 2. source 环境，zsh 或 bash
source devel/setup.zsh[.bash]

# 3. 启动主界面
./run
```

切换到 Sensing 选项卡：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/sensing.png)

### 2.2 回放雷达相机 Bag 

这里回放时需要更改雷达的话题为 `/points_raw`，因为这个工具订阅的雷达主题是固定的：

```shell
rosbag play --pause xxx.bag /rslidar_points:=/points_raw
```

我用的 Robosense 雷达，发布的话题是 `rslidar_points`，这个回放默认暂停，防止跑掉数据，按空格继续或暂停。

### 2.3 启动 Calibration Tool Kit

点击 Calibration Tool Kit 启动标定工具：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/calibration_tool_kit.png)

选择图像输入话题，我只用的 ZED 的左图像话题，如果没有相机话题，确保前面你已经回放了 bag，选择好了点击 OK 确定：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/left_imageraw.png)

选择标定类型为相机到 velodyne 雷达的标定（对 Robosense 雷达也适用，只不过需要更改点云的发布话题），点击 OK 确定：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/camera_velodyne.png)

进入标定主界面 MainWindow：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/main_window_config.png)

配置棋盘格参数：

- Pattern Size(m)：标定板中每个格子的边长，单位 m，我的标定板每个格子长 0.025m
- Pattern Number：标定板长X宽的单元格数量 - 1，我的标定板是长有 12 个格子，宽有 9 个，所以填 11x8，减一是因为标定检测的是内部角点

设置好了后，重启 Calibration Tool Kit，点击左上角 Load 导入第一步标定的相机内参 YAML 文件，但是这个工具只能导入 YML 格式的文件：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/only_yml.png)

因此需要把前面的内参标定文件拷贝一份，修改格式为 yml 即可，YAML 和 YML 其实是一样的：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/yaml2yml.png)

修改好了之后，再点击 Load 加载 yml 格式的内参文件即可：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/load_yml.png)

选择不加载相机和雷达的标定数据，因为我们是直接回放 Bag 标定：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/load_camera_data_no.png)

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/load_cloud_data_no.png)

到这里都设置好了，下面开始外参标定过程！

### 2.4 标定过程

打开回放 bag 终端，按空格继续回放数据，主界面会显示相机图像：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/calibr_play_bag.png)

但是右边的点云窗口没有显示数据，需要我们调整视角才可以，视角的调整方法如下（文末有个 pdf 专门介绍）：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/basic_operations.png)

简单解释下，建议直接操作，很容易：

- 移动点云：上下左右方向键、PgUp、PgDn
- 旋转点云：a、d、w、s、q、e
- 切换模式：数字 1 和数字 2
- 视角缩放：减号缩小、加号放大
- 点云大小：o 键使用小点云、p 使用大点云
- 改变点云窗口背景颜色：b

我使用的使用直接按数字 2 切换模式就能看到点云了：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/mode_2.png)

如果需要更换背景，按 b 键改变为大致灰色即可：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/change_bgcolor.png)

我这里就不改背景了，黑色也挺好看出点云的，然后使用上面的视角操作方法，把点云中的标定板放大到中心位置：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/change_cloud_view2.png)

之后点击右上角的 Grab 捕获当前帧的图像和点云，使用 `-` 和 `+` 缩放视角：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/grab_3.png)

如果你**点击 grab 没反应**很正常，可能是棋盘格离得太远或者模糊了，你多试几个位置应该就能捕获到，我回放一个 Bag 也就捕获了 9 张左右。

然后把鼠标放到右下角捕获的点云窗口，选择一个棋盘格的中心位置区域，关于这个区域的选择，我是参考这个标定工具的文档例子选择的，大概就是标定板的中心位置选择一个圆形的区域，尽量保证向外侧的平面法向量垂直于标定板平面：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/CalibrationToolKitExample.png)

鼠标左键点击选择，右键点击取消，我的选择如下，可以参考：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/select_cloud2.png)

然后重复以上步骤，不断回放暂停，Grab 捕获单帧图像和点云（我一般选择 9 帧左右），选择点云区域，直到回放结束，接着就可以点击右上角的「Calibrate」按钮计算外参矩阵（左上角显示），然后再点击「Project」查看标定效果：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/result_4.png)

切换左下方的单帧图片和点云窗口，捕获的每一帧图像和点云都可以看到对齐效果，另外左边也能看到标定的误差，当然是越小越好，我目前的标定效果一般般，后续打算再标几次。

标定好之后，点击左上角「save」保存外参矩阵即可，文件名建议带上时间戳方便识别：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/save.png)

最后的外参数文件如下，这个文件包含了相机内参和相机到雷达的外参：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ext_mat.png)

以上就是我的雷达相机联合标定过程！希望能帮助正在标定雷达和相机的同学 ^_^！

## 三、标定结果测试

可以直接用之前博客 [Autoware 进行 Robosense-16 线雷达与 ZED 双目相机联合标定！](https://dlonng.com/posts/autoware-calibr-1) 中的「**四、标定结果测试**」一节介绍的步骤来测试融合效果：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/cali_result_test.png)

前几天我把 ROS 的点云和图像的融合节点也调试好了，所以直接在程序里面加载了外参矩阵，然后做了个初步的融合，效果如下：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ros_fusion_result.png)

我也录了个融合视频，可以看看：[B 站：Robosense-16 雷达与 ZED 相机数据融合](https://www.bilibili.com/video/BV1Sp4y1S74w)。

## 五、标定资源

以下是我标定过程中收集的一些好的资料，这里也分享给大家：

- 标定工具的使用文档在这里：[CalibrationToolkit_Manual.pdf](https://github.com/DLonng/AI-Notes/tree/master/SensorFusion/fusion_ws/src/calibration_publisher/docs)
- 这里还有个视频，有条件的同学可以看看：[Yutobe：Autoware 标定相机和雷达](https://www.youtube.com/watch?v=pfBmfgHf6zg)

另外 ROS 融合节点的程序我还在完善中，建议关注我的 Github 项目，后续会上传节点代码：[AI-Notes: lidar_camera_fusion](https://github.com/DLonng/AI-Notes/tree/master/SensorFusion/fusion_ws/src/lidar_camera_fusion)，如果标定遇到问题，可以公众号后台给我发消息，或者直接在博客平台留言，我看到会尽快回复的，不过公众号应该回复的快些，哈哈![img](file:///C:\Users\dlonn\AppData\Local\Temp\SGPicFaceTpBq\7172\190F826C.png)。

