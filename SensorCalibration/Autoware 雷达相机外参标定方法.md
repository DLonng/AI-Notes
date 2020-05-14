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

我打算采用的步骤：

1. 制作标定板
2. 确定标定板的录制位置
   - 6 个站位：近距离（左边近、中间近、右边近）、远距离（左边远、中间远、右边远）、项目用的 16 线雷达比较稀疏所以距离要近点
   - 每个站位做 5 个动作：正向、下俯、上仰、左偏、右偏
3. 在小车的 Ubuntu 下用 rosbag 录制相机 bag1 和雷达、相机的 bag2
4. 拷贝到自己电脑上，用 Autoware 回放 bag1 标定内参，回放 bag2 标定外参

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

重要：[无人驾驶汽车系统入门（二十二）——使用Autoware实践激光雷达与摄像机组合标定：此文章评论区的问题很有价值](https://blog.csdn.net/AdamShan/article/details/81670732#commentBox)

先回放数据 bag，或者连接相机和雷达：

```shell
rosbag play bagName.bag
```

外参标定要使用上一步的内参标定出的 yaml 文件：

```shell
roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:=/PATH/TO/YYYYmmdd_HHMM_autoware_camera_calibration.yaml image_src:=/image
```

- intrinsics_file：相机内参标定文件
- image_src：图像话题

如果启动失败，考虑安装 image-view2：

```shell
sudo apt install image-view2
```

启动成功后，联合标定步骤如下：

1. 启动 Rviz 查看点云图和图像帧
2. 在图像中找到可以匹配点云的像素点
3. 先点击图像中的点，再通过 Rviz 中的 publish point 点击对应的 3D 点云
4. 至少重复 9 个不同点

![](https://img-blog.csdnimg.cn/20190409215537128.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L1hfa2hfMjAwMQ==,size_16,color_FFFFFF,t_70)

寻找 9 组后完成外参标定，结果保存在 home 目录下，也是一个 yaml 文件。

## 五、注意事项

### 5.1 也可以使用分离的 Autoware 标定工具

- [https://zhuanlan.zhihu.com/p/55825255](https://zhuanlan.zhihu.com/p/55825255)

### 5.2 安装位置

此标定工具假定雷达安装时具有雷达传感器的默认坐标轴顺序。

- X 轴指向前方
- Y 轴指向左侧
- Z 轴指向上方

### 5.3 要考虑的重要问题

2. ZED 有左右图像话题，如何外参标定，是每个图像话题单独外参标定一遍？

3. 外参标定如何使用双目相机的内参？双目相机有左右内参

4. 相机和雷达的时间同步问题 [评论区有同步的问题](https://zhuanlan.zhihu.com/p/55825255)

5. 雷达话题转换的重要问题，一定要确定雷达和相机发布的主题名称和标定工具订阅的主题名称相同：

   ```shel
   # autoware 读取的节点是 points_raw，如果发布的节点是 velodnye_points 会读取不出来，需要在发布的时候转换
   rosbag play bagName.bag /velodyne_points:=/points_raw
   ```

   

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