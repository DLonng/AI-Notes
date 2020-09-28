## Lego-loam

```shell
sudo apt-get install libmetis-dev
```

## Robosense

IP：192.168.1.102

## ZED

ok

## IMU

no dev number

Jetson connect IMU 通过 I2C GPIO 口:

- https://docs.nvidia.com/isaac/isaac/doc/tutorials/wire_bmi160_imu.html
- https://platypus-boats.readthedocs.io/en/latest/source/jetson/peripheral/imu.html
- https://www.jetsonhacks.com/2015/04/23/inertial-measurement-unit-imu-part-ii/

## Can

ok

## Opencv

melodic 自带了：https://blog.csdn.net/gloria_littlechi/article/details/85603367

## lidar_camera_fusion

```
set(OpenCV_DIR /usr/share/OpenCV)

// 更改头文件
#include <opencv2/opencv.hpp>
```

## melodic-navigation

```
sudo apt install ros-melodic-tf2-sensor-msgs
```

## 代价地图插件和规划器插件

不能编译通过

