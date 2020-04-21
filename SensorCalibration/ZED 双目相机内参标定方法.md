# ZED 双目相机内参标定方法

今天在家总结了下标定 ZED 相机的步骤，方便开学后直接开整。

## 一、准备工作

- 一个 8x6 的[棋盘标定板](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check-108.pdf)，边长 10.8 cm，因为标定用的是内部角点，所以实际打印出是 9x7 大小
- 保证一个 5m X 5m 的无遮挡环境
- 一个发布了左右图像到 ROS 中的双目相机

## 二、编译和安装驱动

```shell
# rosdep 用来安装指定包的依赖项
rosdep install camera_calibration

# 编译 ROS 标定工具
rosmake camera_calibration
```

## 三、发布图像主题

确保双目相机图片主题已经发布到 ROS 中，使用下面的命令查看：

```shell
rostopic list
```

查看是否有 ZED 的 left 和 right 的 image_raw 主题：

```shell
/my_stereo/left/camera_info
/my_stereo/left/image_raw
/my_stereo/right/camera_info
/my_stereo/right/image_raw
/my_stereo_both/parameter_descriptions
/my_stereo_both/parameter_updates
/my_stereo_l/parameter_descriptions
/my_stereo_l/parameter_updates
/my_stereo_r/parameter_descriptions
```

## 四、启动 ROS 标定工具

键入启动指令运行 Python 标定脚本，添加标定板和图像主题参数：

```shell
rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 0.108 right:=/my_stereo/right/image_raw left:=/my_stereo/left/image_raw right_camera:=/my_stereo/right left_camera:=/my_stereo/left
```

参数解释如下：

- --approximate：摄像机校准器可以处理时间戳不完全相同的图像，当前设置为 0.1 秒，意思是只要时间戳差小于 0.1 秒，标定程序就可以正常运行。
- --size：标定板内角点大小 8x6
- --square：标定板边长 0.108，单位 m
- right：右相机图像主题
- left：左相机图像主题
- right_camera：右相机
- left_camera：左相机

启动界面如下，把棋盘格放到相机视野中，标定过程中 X，Y，Size 会不断变化：

![](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=get&target=stereo_0.png)

## 五、开始标定

### 5.1 水平拿着标定板

确保手拿着标定板长的这一边，不要拿反了：

![](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=get&target=check-108.png)

### 5.2 移动标定板

为了得到好的标定结果，需要在相机视野里面移动标定板，标定板位置如下：

- X 轴标定：移动到视野的最左边，最右边
- Y 轴标定：移动到视野的最上方，最底部
- 倾斜标定：改变标定板的角度，斜着拿
- Size 标定：移动标定板充满整个相机视野
- X，Y 和 Size 一起标定：保持标定板倾斜启动到视野的最左，最右，最上，最下

一些标定画面如下：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/zed_calibration.png)

### 5.3 标定结果

当你移动标定板使得标定程序有了足够的数据计算标定矩阵时，程序中的标定按钮「CALIBRATE」就会变亮，然后点击即可生成标定结果：

![](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=get&target=stereo_1.png)

侧边栏会出现标定精度和尺寸：

![](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=get&target=stereo_2.png)

- epi 为标定精度，当前为 0.16 像素
- dim 为标定尺寸，当前为 0.108 米

在终端会生成左右图像的标定矩阵，这里截取左右相机的内参矩阵 K 和畸变矩阵 distortion：

```shell
Left:
K =  [425.19601412158482, 0.0, 308.29689772295882, 0.0, 426.02702697756899, 223.53789948448997, 0.0, 0.0, 1.0]

Right:
K =  [425.18741896821433, 0.0, 323.8787136811938, 0.0, 426.46867413824884, 226.19589833365407, 0.0, 0.0, 1.0]

[narrow_stereo/left]
distortion
-0.314202 0.091935 0.001295 -0.001620 0.0000

[narrow_stereo/right]
distortion
-0.305505 0.082701 0.001967 -0.001641 0.0000
```

然后点击侧边栏的「SAVE」按钮保存标定结果到 `/tmp/calibrationdata.tar.gz`，至此双目相机内参标定完成！

## 参考博客

- [http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration)
- [https://blog.csdn.net/learning_tortosie/article/details/79901255](https://blog.csdn.net/learning_tortosie/article/details/79901255)







