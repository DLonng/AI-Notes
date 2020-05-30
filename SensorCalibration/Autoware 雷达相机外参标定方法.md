# Robosense - 16 雷达与 ZED 双目相机使用 Autoware 内外参标定方法总结！（未测试）

## 一、Autoware 安装（自己好像已经安装过了）

不使用 Docker 安装，直接源码编译安装：[https://gitlab.com/autowarefoundation/autoware.ai/autoware/-/wikis/Source-Build](https://gitlab.com/autowarefoundation/autoware.ai/autoware/-/wikis/Source-Build)：

### 1.1 官网下载源码（版本 1.10.0 +）

```shell
https://gitlab.com/autowarefoundation/autoware.ai/autoware
```

### 1.2 编译

```shell
cd ~/autoware-版本号/ros
rosdep update

# 安装 ros 依赖
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic 

#使用 catkin_make 编译
./catkin_make_release 
```

### 1.2 启动

```shell
cd autoware.ai

source devel/setup.bash

rosrun autoware_camera_lidar_calibrator cameracalibrator.py [启动参数]
```

参考链接：

- [https://zhuanlan.zhihu.com/p/98979334](https://zhuanlan.zhihu.com/p/98979334)
- [https://gitlab.com/autowarefoundation/autoware.ai/autoware/-/wikis/Source-Build](https://gitlab.com/autowarefoundation/autoware.ai/autoware/-/wikis/Source-Build)

## 二、标定准备

### 2.1 标定板

使用 9 X 7 的[标定板](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check-108.pdf)，规格为 A0 尺寸的硬纸板(841 × 1189mm) 9 行 7 列，注意要使用硬纸板，软的纸板你举不平整：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/checkboard.png)

### 2.2 录制标定包（看情况是否录制）

如果你不想在标定时连接相机和雷达，可以提前录制好双目相机和雷达的数据 bag，如果打算连接硬件就跳过这一步。

使用 Autoware 录制步骤：

1. 打开 Autoware 中的 ROSBAG 录包工具
2. 选择 `/image_raw` 和 `/velodny_points` 这两个话题
3. 点击 start 启动录制，然后保存

如下图：

![图片待补充]()

### 2.3 基本步骤

1. 制作标定板

2. 确定标定板的录制位置（要考虑站的距离不要太远）
   - 6 个站位：近距离（左边近、中间近、右边近）、远距离（左边远、中间远、右边远）、项目用的 16 线雷达比较稀疏所以距离要近点
   - 每个站位做 5 个动作：正向、下俯、上仰、左偏、右偏
   
   ![](https://img-blog.csdn.net/2018081417140238?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

3. 在小车的 Ubuntu 下用 rosbag 录制相机话题的 `zed_calibration.bag` 用于内参标定
4. 在小车的 Ubuntu 下用 rosbag 录制雷达话题、相机话题的 `zed_lidar_calibration.bag` 用于外参标定
5. 拷贝 2 个 bag 到自己电脑上，用 Autoware 回放 bag1 标定内参，回放 bag2 标定外参
6. 保存内参参数文件

## 三、相机内参标定（相机出厂自带？）

下面开始标定工作，外参标定需要使用相机的内参，所以先来标定内参，如果录制了 bag 则回放数据（按空格暂停回放）：

```shell
# 查看数据包中的话题名
rosbag info bagName.bag

# topic_name 为相机的话题名
rosbag play bagName.bag --topic /topic_name
```

没有录制就直接连接相机，ZED 双目相机的内参标定方法可以使用 ROS 提供的工具，参考之前的这篇文章：[ZED 双目相机内参标定方法]()。

在 Autoware 中提供标定**单目和双目**相机内参的工具，步骤和之前文章介绍的基本相同，这里列下大概的步骤：

### 3.1 启动标定工具

标定单目相机内参命令：

```shell
rosrun autoware_camera_lidar_calibrator cameracalibrator.py --square SQUARE_SIZE --size MxN image:=/image_topic
```

标定双目相机内参命令：

```shell
rosrun autoware_camera_lidar_calibrator cameracalibrator.py --square SQUARE_SIZE  --size MxN right:=/image_topic left:=/image_topic
```

命令参数的解释和后续的标定步骤与之前的文章一样：[ZED 双目相机内参标定方法]()。

### 3.2 注意事项

- 标定文件保存在 /home 目录下，名称类似于 20190401_1133_autoware_camera_calibration.yaml。
- Save 的时候报了个错说没有 cv2 有关的那个对象没 write 方法，然后 pip 重新装一下 opencv-python 即可？待确认
- 联合标定不能使用双目标定的内参文件，只能使用单目标定出的 yaml 文件？待确认

## 四、雷达相机外参标定

### 4.1 第一种方法

参考博客：

- [使用 Autoware 进行 (双目) 相机与激光雷达的联合标定](https://blog.csdn.net/X_kh_2001/article/details/89163659?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task)
- [激光雷达和相机的联合标定（Camera-LiDAR Calibration）之 Autoware](https://blog.csdn.net/learning_tortosie/article/details/82347694)

(1) 先回放雷达和相机的数据 `lidar_zed_calibration.bag`：

```shell
# 查看发布的主题信息
rosbag info zed_lidar_calibration.bag
```

开始回放，**是否需要更改发布的点云话题名？**：

```shell
rosbag play zed_lidar_calibration.bag [/xxx_points:=/points_raw]
```

注意执行完命令后**立刻按空格暂停**，因为我们要在后面标定的时候才回放完整的标定包！

(2) 启动 autoware 标定工具 `autoware_camera_lidar_calibrator`，外参标定要使用相机的内参标定出的 yaml 文件：

```shell
roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:=/PATH/TO/YYYYmmdd_HHMM_autoware_camera_calibration.yaml image_src:=/image
```

- `intrinsics_file`：ZED 相机内参标定文件，ZED 有 2 个摄像头怎么办？指定单个相机的内参文件。
- `image_src`：图像话题，ZED 有 2 个摄像头怎么办？指定单个相机话题 `image_src:=/stereo/left/image_raw`

启动成功后会打开一个图像查看器：

![image-20200517141443229](/Users/mac/Library/Application Support/typora-user-images/image-20200517141443229.png)

如果启动失败，考虑安装 image-view2 组件，然后重新启动：

```shell
sudo apt-get install image-view2

sudo apt-get install ros-kinetic-jsk-common
```

(3) 启动成功后，联合标定具体步骤如下：

1. 启动 RVIZ 订阅点云话题和图像话题
2. 在图像中**手动找到**一个可以匹配点云的像素点
3. 先点击图像中的点，再通过 RVIZ 中的 publish point 点击对应的 3D 点
4. 至少重复 9 个不同点完成外参标定

![](https://img-blog.csdnimg.cn/20190409215537128.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L1hfa2hfMjAwMQ==,size_16,color_FFFFFF,t_70)

(4) 最后的外参结果保存在 home 目录下，名称为 `YYYYmmdd_HHMM_autoware_lidar_camera_calibration.yaml`，此文件可与 Autoware 的 Calibration Publisher 一起使用，以发布和对齐 LiDAR 与相机之间的转换。

该外参文件的参数如下：

![image-20200517103825143](/Users/mac/Library/Application Support/typora-user-images/image-20200517103825143.png)



### 4.2 [第二种方法 Calibration Tool Kit

参考博客：

- [无人驾驶汽车系统入门（二十二）—— 使用 Autoware 实践激光雷达与摄像机组合标定](https://blog.csdn.net/AdamShan/article/details/81670732)
- [Autoware 搭建自动驾驶系统。一：数据记录 / 播放和传感器校准](https://blog.csdn.net/jianxuezixuan/article/details/87283600?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-17.nonecase&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-17.nonecase)

(1) 用命令回放 bag：

```shell
# 查看发布的主题信息
rosbag info zed_lidar_calibration.bag

/[图像话题名]
/[点云话题名]
```

看清楚发布的图像话题和点云话题名，类似这样的信息 `topics`：

![image-20200517143504878](/Users/mac/Library/Application Support/typora-user-images/image-20200517143504878.png)

然后回放这个包，注意可能需要**重命名输入点云的话题名，因为 autoware 标定工具的输入点云话题名为 `/points_raw`**：

```shell
rosbag play zed_lidar_calibration.bag /xxx_points:=/points_raw
```

注意执行完命令后**立刻按空格暂停**，因为我们要在后面标定的时候才回放完整的标定包！

(2) 启动标定工具箱

![image-20200517105144295](/Users/mac/Library/Application Support/typora-user-images/image-20200517105144295.png)

选择输入图像话题名，需要修改：

![image-20200517142915593](/Users/mac/Library/Application Support/typora-user-images/image-20200517142915593.png)

选择输入点云的话题名，修改修改：

![image-20200517142945884](/Users/mac/Library/Application Support/typora-user-images/image-20200517142945884.png)

(3) 设置参数（参数含义有待确定）：

- Pattern Size (m)：0.108 和 0.108
- Pattern Number：6 列和 8 行

![image-20200517144549248](/Users/mac/Library/Application Support/typora-user-images/image-20200517144549248.png)

或者设置为 0.13 和 0.13 也行，这个是标定板的长宽吗？

![image-20200517144723266](/Users/mac/Library/Application Support/typora-user-images/image-20200517144723266.png)

这些参数根据自己的标定板设置，**设置完后，重启标定工具**！

(4) 开始标定

首先左上角 load 之前标定的相机内参文件，导入相机内参：

![image-20200517145723215](/Users/mac/Library/Application Support/typora-user-images/image-20200517145723215.png)

开始回放 bag，当出现图像画面时，按空格暂停回放，并调整点云的视角，确保图像和点云都可以看到完整的白标定板：

![](https://img-blog.csdn.net/20180814171727556?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

点云视角的调整参考这个博客：[点云视角调整](https://blog.csdn.net/AdamShan/article/details/81670732)，调整视角使得右边的激光雷达窗口可以看到标定板的点云，然后点击右上角的 Grab 捕获单帧图片和点云，捕获的结果出现在下方：

![](https://img-blog.csdn.net/2018081417182412?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

在捕获单帧的点云中，选取图片中对应标定板的位置，选取的是**圆圈内的所有点**，所包含的信息不仅仅只有点，还有平面法向量，标定的时候**一定要确保法向量与平面是垂直**。

在右下方的点云帧中找到标定板对应的位置，鼠标左键标记，如果标记有误，鼠标右键取消：

![](https://img-blog.csdn.net/20180814171847963?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

不断重复以上步骤捕获和标记步骤，直到回放结束。然后点击右上角的 Calibrate 按钮进行外参矩阵的计算，CPU 会占满一段时间：

![image-20200517172324965](/Users/mac/Library/Application Support/typora-user-images/image-20200517172324965.png)

标定好之后，再点击的 project，可以查看标定的效果，左下方图像中会出现根据计算结果和激光雷达数据生成的图像对应位置，以红色散点表示，如果散点分布在标定板上，说明标定正确，一般可以看到如下效果：

![](https://img-blog.csdn.net/20180814171900690?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)



![](https://pic1.zhimg.com/80/v2-a174d9bdf7d51cd55088e2649bfdb25c_720w.jpg)

![](https://pic1.zhimg.com/80/v2-8f1c5669dfb5349a2ae3475fd63fd90c_720w.jpg)

如果散点不在标定板上，则需要重新标定：

![](https://img-blog.csdn.net/20180814171911776?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

标定无误后，点击右上角的 Save 保存结果，出现的 2 个弹窗都选 NO：

![](https://img-blog.csdn.net/20180814171922331?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)



![](https://img-blog.csdn.net/20180814171933181?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)



### 4.3 测试标定结果

使用 Autoware 自带的 Calibration Publisher 和 Point Image 节点获得点云 - 图像融合的 point-image 结果：

- 打开 Autoware 界面
- 在 `Autoware/Sensing/` 中打开 `Calibration Publisher`，并将标定好的相机内参和相机雷达外参文件读取进来
- 打开 Autoware/Sensing 下的 `Point Image` 节点
- 打开 Rviz，在 Panels/Add New Panel 内添加 `ImageViewerPlugin`，填写 Image Topic，本例为 `/image_raw`，再添加 Point Topic，本例为 `/point_image`

![image-20200517104327303](/Users/mac/Library/Application Support/typora-user-images/image-20200517104327303.png)

得到如下融合图：

![image-20200517104705813](/Users/mac/Library/Application Support/typora-user-images/image-20200517104705813.png)

### 4.4 外参矩阵的使用

因为上面标定的外参关系是**相机 -> 雷达**，而我们在实际融合时需要把点云投影到图像上，也即雷达坐标系转换到相机坐标系，因此需要对标定的外参矩阵进行**求逆**！

## 五、注意事项

### 5.1 也可以使用分离的 Autoware 标定工具

- [https://zhuanlan.zhihu.com/p/55825255](https://zhuanlan.zhihu.com/p/55825255)

### 5.2 安装位置

此标定工具假定雷达安装时具有雷达传感器的默认坐标轴顺序。

- X 轴指向前方
- Y 轴指向左侧
- Z 轴指向上方

## 六、可能遇到的重要问题

1. 验证融合结果不显示点云：更改坐标系？检查话题名？
2. 记得 `source devel/setup.bash`！
3. 标定板去打印店打印即可，用泡沫板固定。
4. 外参标定使用的相机内参为 yaml 格式的文件，如果相机的内参不是该格式，可以用 autoware 自带的内参标定工具重新标定一次，不是很麻烦。
5. grab 不能出画面：因为标定板在图像中占用的比例太小，把标定板离相机近一点就可以检测到角点。
6. 外参标定如何使用双目相机的内参：对左右图像单独进行外参标定，标定那个就用那个相机的内参文件？
7. 相机和雷达的时间同步问题 [评论区有同步的问题](https://zhuanlan.zhihu.com/p/55825255)
8. 雷达话题转换的重要问题，一定要确定雷达和相机发布的主题名称和标定工具订阅的主题名称相同：

```shel
# autoware 读取的节点是 points_raw，如果发布的节点是 velodnye_points 会读取不出来，需要在发布的时候转换
rosbag play bagName.bag /velodyne_points:=/points_raw
```

## 七、评论区有价值的文章

- https://blog.csdn.net/AdamShan/article/details/81670732#commentBox

## 参考博客

- [https://blog.csdn.net/learning_tortosie/article/details/82347694](https://blog.csdn.net/learning_tortosie/article/details/82347694)
- https://blog.csdn.net/AdamShan/article/details/81670732
- [https://blog.csdn.net/X_kh_2001/article/details/89163659?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task](https://blog.csdn.net/X_kh_2001/article/details/89163659?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task)
- [https://zhuanlan.zhihu.com/p/55825255](https://zhuanlan.zhihu.com/p/55825255)
- [此文章评论区的问题很有价值](https://blog.csdn.net/AdamShan/article/details/81670732#commentBox)

新添加：

- [使用autoware获得相机内参并与雷达联合标定：标定双目、外参矩阵求逆问题](https://blog.csdn.net/Mr_yangsir/article/details/101013639)
- [Autoware完整安装及联合标定工具箱安装](https://blog.csdn.net/qq_42615787/article/details/102481314)
- [Autoware搭建自动驾驶系统.一：数据记录/播放和传感器校准](https://blog.csdn.net/jianxuezixuan/article/details/87283600?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-17.nonecase&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-17.nonecase)






