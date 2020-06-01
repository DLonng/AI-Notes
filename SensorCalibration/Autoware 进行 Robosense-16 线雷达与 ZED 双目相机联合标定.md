## 一、编译安装 Autoware-1.10.0

我没有安装最新版本的 Autoware，因为新版本不带雷达和相机的标定工具，我安装的是 1.10.0 版本！

### 1.1 下载 Autoware-1.10.0 源码

不建议[官方的 git check 安装方式](https://gitlab.com/autowarefoundation/autoware.ai/autoware/-/wikis/Source-Build)，因为不熟悉 git 可能会遇到问题，直接在[Gitlab 仓库](https://gitlab.com/autowarefoundation/autoware.ai/autoware)选择 1.10.0 版本下载即可：

![下载 Autoware 源码](C:\Users\dlonn\Desktop\标定博客图片\下载 Autoware 源码.png)

### 1.2 编译 Autoware-1.10.0

编译过程比较容易，我也没遇到编译错误，解压下载的 autoware-1.10.0，在该目录下执行以下命令：

```shell
# 1. 进入 autoware 的 ros 目录下
cd autoware-1.10.0/ros

# 2. rosdep 安装依赖
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO


# 3. 编译
./catkin_make_release
```

我的台式机配置比较低，大概编译了 1 个小时，好的配置应该编译的更快。

### 1.3 启动 Autoware-1.10.0

首先还是要进入 ros 目录下，然后 source 环境，之后执行 `run` 程序即可启动主界面：

```shell
# 1. 进入 autoware 的 ros 目录下
cd autoware-1.10.0/ros

# 2. source 环境，zsh 或 bash
source devel/setup.zsh[.bash]

# 3. 启动主界面
./run
```

可能需要输入 root 密码，然后启动的主界面如下：

![主界面](C:\Users\dlonn\Desktop\标定博客图片\主界面.png)

至此 Autoware 就安装好了，后面我们需要用它提供的标定工具包来进行内参和外参的标定，以及标定结果的融合效果测试。

## 二、标定 ZED 相机内参

### 2.1 内参标定准备

内参标定需要准备标定板，我用的是我们实验室自己购买的 12x9，棋盘格为 2.5cm 的专业标定板，比较精准，如下：

![标定板](C:\Users\dlonn\Desktop\标定博客图片\标定板.jpg)

然后录制一个相机左右话题的 Bag：

```shell
rosbag record -O zed_calibration.bag /camera/left/image_raw /camera/right/image_raw
```

为了得到好的标定结果，录制过程中需要在相机视野里面移动标定板，建议位置如下：

- X 轴标定：移动到视野的最左边，最右边
- Y 轴标定：移动到视野的最上方，最底部
- 倾斜标定：改变标定板的角度，斜着拿
- Size 标定：移动标定板充满整个相机视野
- X，Y 和 Size 一起标定：保持标定板倾斜启动到视野的最左，最右，最上，最下

然后我拷贝 Bag 到台式机上回放，但是有问题提示需要 `rogbag reindex`：

```shell
rosbag reindex zed_calibration.bag
```

执行修复下就 OK，速度很快，不过后面的数据会少一些，可能是拷贝过程中的错误导致的，无伤大雅。

### 1.2 内参标定过程

内参标定比较简单，基本都是自动执行，先 source Autoware 环境以使用标定工具：

```shell
cd autoware-1.10.0/ros/

source devel/setup.zsh
```

启动 roscore：

```shell
roscore
```

启动标定工具 `autoware_camera_lidar_calibrator`，但是这个工具同时标定双目得到的标定 YAML 文件不能直接作为后面外参标定的输入，因为文件格式有些不同，我也是做实验发现的，因此我单独标定左右相机，这样就会生成可用的 Autoware 格式的 YAML 文件：

```shell
rosrun autoware_camera_lidar_calibrator cameracalibrator.py --square 0.025  --size 11x8  image:=/camera/left/image_raw
```

参数如下：

- `--square`：标定板单元格的边长（m），我的标定板是 2.5cm，也就是 0.025m
- `--size`：标定板长x宽的格子数减一，我的标定板是 12x9，所以填 11x8
- `image`：要标定的相机话题，左或者右

启动后就是一个黑窗口：

![zed_display](C:\Users\dlonn\Desktop\AI-Notes\SensorCalibration\标定博客图片\zed_display.png)

然后开始回放内参标定 Bag，默认暂停启动，按空格继续：

```shell
rosbag play --pause zed_calibration.bag
```

标定过程如下，标定工具会根据棋盘格位置自动检测角点：



当右上角的 X、Y、Size、Skew 变为绿色时，标定按钮「CALIBRATE」可用，点击即可计算内参矩阵：



结果在 Shell 中打印出来，点击「SAVE」可保存到 home 目录下：



注意这里会多保存一个 Autoware 类型的 YAML 文件格式，也就是后面外参标定要导入的文件！内容如下：





## 三、ZED 相机和 Robosense-16 线雷达联合标定

### 3.1 联合标定准备

联合标定也要准备标定板和录制 bag 包，标定板用的也是内参标定的棋盘格，另外因为我是在电脑上安装的 Autoware，所以需要在小车上录制雷达和相机的 Bag 数据包，然后再拷贝到我的电脑上回放用于标定工具的话题输入。

我录制 bag 包的命令如下，录制的是ZED 左右相机话题、雷达话题：

```shell
rosbag record -O zed_lidar_calibration.bag /camera/left/image_raw /camera/right/image_raw /rslidar_point
```

但是录制完后，我拷贝到台式机上，回放错误，提示我要 `reindex` 以下，我问我同学也遇到过这个问题，暂时不知道为何，不过 reindex 后 Bag 可以正常回放：

```shell
rosbag reindex zed_lidar_calibration.bag
```

查看下 info，没有问题：

```shell
rosbag info
```

![rosbag_info](C:\Users\dlonn\Desktop\标定博客图片\rosbag_info.png)

回放 Bag 使用如下命令，加上 `--pause` 意思是启动即暂停，防止跑掉数据，按空格继续回放：

```shell
rosbag play --pause zed_lidar_calibration.bag
```

下面我们开始使用 `autoware_camera_lidar_calibrator` 工具标定雷达和相机。

### 3.2 标定过程

首先启动 roscore：

```shell
roscore
```

接着初始化 Autoware：

```shell
cd autoware-1.10.0/ros/

source devel/setup.zsh
```

然后启动标定工具，这里我标定 ZED 左相机图像和雷达，使用右相机同理：

```shell
roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:=xxx.yaml image_src:=/camera/left/image_raw
```

- `intrinsics_file`：前面标定 ZED 的内参文件
- `image_src`：要标定的相机话题，这里用的 left image，有需要也可以用 right image

遇到的第一个错误，启动失败提示找不到 `image-view2`，直接 apt 安装：

```shell
sudo apt-get install ros-kinetic-jsk-common
```

- 参考链接：[https://github.com/jsk-ros-pkg/jsk_common](https://github.com/jsk-ros-pkg/jsk_common)

遇到的第二个错误，提示找不到 `libopencv_core3.so.3.3`：

![opencv_core3](C:\Users\dlonn\Desktop\标定博客图片\opencv_core3.png)

我在系统中查找 `libopencv_core3.so` 这个库：

```shell
locate libopencv_core3.so
```

发现它在如下位置：

```shell
/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so
```

然后我这个目录下的所有 opencv 库复制一份到上一级 lib 目录下，解决了这个问题：

```shell
sudo cp /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_* /opt/ros/kinetic/lib
```

- 参考博客：[https://blog.csdn.net/qifengle315/article/details/103434598](https://blog.csdn.net/qifengle315/article/details/103434598)

之后我就可以启动这个标定工具了，界面如下就是一个图片查看器：

![image-view2](C:\Users\dlonn\Desktop\标定博客图片\image-view2.png)

然后开始回放 Bag 数据，记得按空格开始回放：

```shell
rosbag play --pause zed_lidar_calibration.bag
```

上面的 image-view2 就会出现相机画面，然后我们按空格暂停回放，准备标定：

![image-view2-bag](C:\Users\dlonn\Desktop\标定博客图片\image-view2-bag.png)

标定还需要启动 rviz：

```shell
rosrun rviz rviz
```

点击 Add 添加要订阅的 Image 和 PontCloud2 话题：

![add_rviz_topic](C:\Users\dlonn\Desktop\标定博客图片\add_rviz_topic.png)

分别设置每个订阅话题的 topic、FixedFrame 设置为 rslidar 不然会没有点云显示、切换点云查看视角，用鼠标滑轮调整点云距离，确保能看到我这样的标定板：

![rviz_rslidar_zed](C:\Users\dlonn\Desktop\标定博客图片\rviz_rslidar_zed.png)

然后我们同时切换出 image-view2 的界面，点击工具栏放大图像，然后按照如下步骤手动选择一个像素点和点云进行单次标定：

1. 观察图像和点云，并在 image-view2 中用鼠标选择一个像素点
2. 点击 rviz 工具栏的 Publish Point
3. 然后在 rviz 中选择一个对应的点云数据点，当你的鼠标右下角出现一个浅红色的路标记号时即可点击该数据点
4. 观察 image-view2 的窗口是否出现 points 的提示信息

![calibr_process](C:\Users\dlonn\Desktop\标定博客图片\calibr_process.png)

重复以上步骤，选择 9 个不同的像素-点云对，因为需要足够的数据才能计算外参矩阵，当第 9 个点选择完后，该工具会自动计算外参标定矩阵：

![nine_calibr](C:\Users\dlonn\Desktop\标定博客图片\nine_calibr.png)

最终的标定文件保存在 home 目录下，以下是外参文件内容，第一个就是 4x4 的外参矩阵：

![calibr_result](C:\Users\dlonn\Desktop\标定博客图片\calibr_result.png)

### 3.3 标定结果测试

标定矩阵有了之后，我们来利用 autoware 提供的融合工具来看下标定的效果如何，先来回放数据：

```shell
rosbag play --pause zed_lidar_calibration.bag /rslidar_points:=/points_raw
```

这里要把雷达的话题换成 `points_raw`，因为 autoware 订阅的话题名是这个！然后启动 Autoware 主界面，启动方法跟前面一样，切换到 Sensing 标签页，配置如下：

- Camera ID：我选择的是 left 图像
- target_frame：默认 velodyne 即可，因为我们已经将雷达话题名改为 velodyne 订阅的名字
- Ref：选择上一步的外参标定文件
- image topic source：因为 Camera ID 已经指定了，所以这里只需要填 topic 名即可

![calibr_publish](C:\Users\dlonn\Desktop\标定博客图片\calibr_publish.png)

点击 OK 关闭窗口（查看终端是否会输出红色错误信息，一般不会），然后再点击 Points Image 选择相机 ID 为 left，点击 OK 确定（此时终端再输出一些信息，但不会报红色错误），如果你的终端出现红色错误信息，就要查看配置是否正确了：

![points_image](C:\Users\dlonn\Desktop\标定博客图片\points_image.png)

再点击下面的 Rviz 启动 rviz，注意不要单独在终端中 rosrun 启动 rviz，单独启动没有 image-view2 的插件，在 autoware 中启动提供融合的插件：

![add_image_view_panel](C:\Users\dlonn\Desktop\标定博客图片\add_image_view_panel.png)

进行如下选择：

- Image Topic：`/camera/left`
- Point Topic：`/points_image`

然后切换到回放 Bag 终端，按空格继续回放数据，即可出现融合效果，我这里效果一般般，后面打算再重新标定：

![fusion_result](C:\Users\dlonn\Desktop\标定博客图片\fusion_result.png)

OK！以上就是我的雷达相机内外参标定总结！

