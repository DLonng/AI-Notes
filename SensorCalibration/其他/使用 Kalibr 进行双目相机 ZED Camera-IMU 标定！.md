# 使用 Kalibr 进行双目相机 ZED Camera-IMU 标定！（未测试）

## 一、标定目的

标定 IMU 和相机的目的是为了获取 IMU 坐标系和相机坐标系的相对位姿矩阵 `T_cam_imu`。

## 二、安装 Kalibr 标定工具

使用 [Kalibr](https://github.com/ethz-asl/kalibr) 进行标定，官方安装步骤：[https://github.com/ethz-asl/kalibr/wiki/installation](https://github.com/ethz-asl/kalibr/wiki/installation)。

这里总结下我自己在 Ubuntu 16.04 上的安装过程，建议通过源码编译安装全部功能（CDE 包安装功能不全，且只支持 64 bit）。

### 2.1 安装依赖

注意要将其中的 ROS 版本 kinect 替换成你使用的版本名称：

```shell
sudo apt install python-setuptools python-rosinstall ipython libeigen3-dev libboost-all-dev doxygen libopencv-dev ros-kinect-vision-opencv ros-kinect-image-transport-plugins ros-kinect-cmake-modules python-software-properties software-properties-common libpoco-dev python-matplotlib python-scipy python-git python-pip ipython libtbb-dev libblas-dev liblapack-dev python-catkin-tools libv4l-dev

sudo pip install python-igraph --upgrade
```

如果 pip 未安装会报错，直接 apt 安装 pip 即可，注意自己的 Python 版本：

```shell
# Python 2
sudo apt install python-pip

# Python 3
sudo apt install python3-pip
```

### 2.2 创建 catkin 工作空间

```shell
mkdir -p ~/kalibr_workspace/src
cd ~/kalibr_workspace

source /opt/ros/kinetic/setup.bash

catkin init
catkin config --extend /opt/ros/kinetic

# Necessary for catkin_tools >= 0.4. catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 2.3 下载源码编译

直接拉下仓库：

```shell
cd ~/kalibr_workspace/src
git clone https://github.com/ethz-asl/Kalibr.git
```

编译，如果电脑性能不强，可以去掉 `-j4`，或者换成 2 线程 `-j2`：

```shell
cd ~/kalibr_workspace
catkin build -DCMAKE_BUILD_TYPE=Release -j4
```

### 2.4 使用 Kalibr

编译成功后，在当前目录的 `/devel/bin` 目录下会生成标定程序，我们 source 一下 Kalibr 的环境变量就可以开始标定了：

```shell
source ~/kalibr_workspace/devel/setup.bash
```

## 三、Camera - IMU 标定过程

### 3.1 准备标定数据

Camera - IMU 的标定需要使用 4 个输入文件：

- `--bag filename.bag`：包含 image 和 IMU 的 ROS bag
- `--cam camchain.yaml`：双目相机内外参
- `--imu imu.yaml`：IMU 参数文件
- `--target target.yaml`：标定板参数文件

下面分别详细介绍这 4 个文件。

#### 3.1.1 filename.bag

在标定前需要录制在连续时间内拍摄标定板的图像和 IMU 数据，然后利用自带的 `kalibr_bagcreate` 转换成 ROS bag，bag 包结构如下：

```shell
+-- dataset-dir
    +-- cam0
    │   +-- 1385030208726607500.png
    │   +--      ...
    │   \-- 1385030212176607500.png
    +-- cam1
    │   +-- 1385030208726607500.png
    │   +--      ...
    │   \-- 1385030212176607500.png
    \-- imu0.csv
```

IMU 的 CSV 文件如下：

```shell
timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z
1385030208736607488,0.5,-0.2,-0.1,8.1,-1.9,-3.3
 ...
1386030208736607488,0.5,-0.1,-0.1,8.1,-1.9,-3.3
```

制作 ROS bag 包命令如下：

```shell
kalibr_bagcreater --folder dataset-dir --output-bag fileaname.bag
```

- `dataset-dir`：输入数据路径
- `filename.bag`：输出的 ROS bag 名称，默认在当前路径

从 ROS bag 导出话题命令如下：

```shell
kalibr_bagextractor --image-topics /cam0/image_raw /cam1/image_raw --imu-topics /imu0 --output-folder dataset-dir --bag filename.bag
```

#### 3.1.2 camchain.yaml

文件编写格式：[yaml-formats](https://github.com/ethz-asl/kalibr/wiki/yaml-formats)，解释如下：

- camera_model：相机类型（pinhole / omni），如何选择？
- intrinsics：相机内参，pinhole: [fu, fv, pu, pv]，omni: [xi, fu, fv, pu, pv]，如何选择？
- distortion_model：畸变模型（radtan / equidistant），如何选择？
- distortion_coeffs：畸变系数
- T_cam_imu：左右相机相对位姿，还需要标定！
- timeshift_cam_imu：捕获相机和 IMU 数据的时间偏移
- rostopic：ROS 相机图像主题
- resolution：相机分辨率 [width, height]

```yaml
cam0:
  camera_model: pinhole
  intrinsics: [461.629, 460.152, 362.680, 246.049]
  distortion_model: radtan
  distortion_coeffs: [-0.27695497, 0.06712482, 0.00087538, 0.00011556]
  timeshift_cam_imu: -8.121e-05
  rostopic: /cam0/image_raw
  resolution: [752, 480]
cam1:
  camera_model: omni
  intrinsics: [0.80065662, 833.006, 830.345, 373.850, 253.749]
  distortion_model: radtan
  distortion_coeffs: [-0.33518750, 0.13211436, 0.00055967, 0.00057686]
  T_cn_cnm1:
  - [ 0.99998854, 0.00216014, 0.00427195,-0.11003785]
  - [-0.00221074, 0.99992702, 0.01187697, 0.00045792]
  - [-0.00424598,-0.01188627, 0.99992034,-0.00064487]
  - [0.0, 0.0, 0.0, 1.0]
  timeshift_cam_imu: -8.681e-05
  rostopic: /cam1/image_raw
  resolution: [752, 480]
```

#### 3.1.3 imu.yaml

需要参考 IMU 的数据手册确定参数：

- Accelerometers：加速度计参数
- Gyroscopes：陀螺仪参数
- rostopic：IMU 的 ROS 主题
- update_rate：IMU 更新频率

```yaml
# Accelerometers
accelerometer_noise_density: 1.86e-03   # Noise density (continuous-time)
accelerometer_random_walk:   4.33e-04   # Bias random walk

# Gyroscopes
gyroscope_noise_density:     1.87e-04   # Noise density (continuous-time)
gyroscope_random_walk:       2.66e-05   # Bias random walk

rostopic:                    /imu0      # the IMU ROS topic
update_rate:                 200.0      # Hz (for discretization of the values above)
```

#### 3.1.4 target.yaml

Kalibr 支持 3 种[标定板](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)：

- Aprilgrid：四月网格？
- Checkerboard：棋盘格
- Circlegrid：圆网格

如果使用具有对称性的校准目标（棋盘格，圆网格），则必须避免可能导致目标姿态估计值发生翻转的移动，所以官方建议使用 Aprilgrid 完全避免此问题。

标定板参数见：[https://github.com/ethz-asl/kalibr/wiki/calibration-targets](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)

### 3.2 启动标定程序

准备好以上 4 个输入数据后，就可以启动标定程序：

```shell
kalibr_calibrate_imu_camera --bag [filename.bag] --cam [camchain.yaml] --imu [imu.yaml] --target [target.yaml] --bag-from-to 5 45
```

- `--bag-from-to 5 45`：是要使用 bag 数据的时间段，单位 S

标定的输入结果如下：

- `report-imucam-%BAGNAME%.pdf`：PDF 的标定报告

- `results-imucam-%BAGNAME%.txt`：文本文档的结果

- `camchain-imucam-%BAGNAME%.yaml`：该文件在输入的 camchain.yaml 基础上增加了 Camera-IMU 的标定结果 `T_cam_imu` 矩阵

  ```yaml
  cam0:
  		...
      T_cam_imu:
      - [0.01779318, 0.99967549,-0.01822936, 0.07008565]
      - [-0.9998017, 0.01795239, 0.00860714,-0.01771023]
      - [0.00893160, 0.01807260, 0.99979678, 0.00399246]
      - [0.0, 0.0, 0.0, 1.0]
      ...
      
  cam1:
  		...
      T_cam_imu:
      - [ 0.01567142, 0.99978002,-0.01393948,-0.03997419]
      - [-0.99966203, 0.01595569, 0.02052137,-0.01735854]
      - [ 0.02073927, 0.01361317, 0.99969223, 0.00326019]
      - [0.0, 0.0, 0.0, 1.0]
      ...
  ```

  

## 四、注意事项

- pip 找不到，安装 pip

- numpy 找不到，安装 numpy

  ```shell
  # Python 2
  sudo apt install python-numpy
  
  # Python 3
  sudo apt install python3-numpy
  ```

- 录制标定数据时动作不要太快，防止图像模糊，充分旋转和加速 IMU，并遍历所有轴

- Kalibr 也可以标定单目相机，如何修改参数？

- 建议使用 Aprilgrid 标定板

## 参考博客

- [https://blog.csdn.net/zhubaohua_bupt/article/details/80222321](https://blog.csdn.net/zhubaohua_bupt/article/details/80222321)
- [https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)

