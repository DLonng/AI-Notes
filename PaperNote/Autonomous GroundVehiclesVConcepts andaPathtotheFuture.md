# Autonomous Ground Vehicles - Concepts and a Path to the Future

## Perception

### Vehicle State Estimation

作为感知和控制模块的先决条件，必须对车辆的运动进行良好的估算。特别是当车辆快速行驶或在非平坦地形上时，会沿纵向和横向轴发生相应的旋转。

### Static Obstacles

为了处理环境中的静态障碍物，通常使用占用栅格映射，来自 LIDAR 点云或立体摄像机深度图像的数据被融合成网格。

### Moving Obstacles

基于 LIDAR 的移动物体检测与跟踪：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/LIDAR_tracking.png)

基于 Image 的移动物体检测与跟踪：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/visual-tracking.png)

### Road Shape Estimation

车道线检测：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/road_shaped_estimation.png)

### Map-Aided Localization

如果已知某种类型的环境地图，则可以通过将地图特征与本地感测到的特征相关联并确定偏移量来改善定位。