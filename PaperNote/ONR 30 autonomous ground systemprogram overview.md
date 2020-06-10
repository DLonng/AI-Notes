# ONR 30 autonomous ground system program overview

## 地面平台

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/MAGV.png)

## 环境感知

为了在任何地形上成功地自主导航，车辆必须了解其姿势，方向以及在穿越路线时必须回避的障碍物的位置，此外，对于要求系统保持在规定车道内的任务，明确检测道路和路径至关重要，感知需要实时融合来自车载传感器的数据，以提取有关车辆环境及其在环境中的位置和方向的信息。

### 感知传感器

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/MAGV_perception_sensor.png)

- ColorCamera
- LIDAR

### 感知的关键技术

- 雷达相机外参标定
- 障碍物检测
- 地形分类
- 粗糙度检测
- 导航地图构建

将感知数据融合到世界模型中，导航使用感知方面的关键技术包括用于 LWIR 摄像机的新型校准方法和外部 LIDAR 到摄像机的校准，障碍物检测，地形分类，粗糙度检测，立体地图评估以及将感知数据融合到世界模型，供路径规划使用。

