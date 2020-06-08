# A perception pipeline for expeditionary autonomous ground vehicles

## 感知管道框架

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/perception_pipeline_2.png)

## 传感器标定

- 相机内参
- 雷达相机外参

## 传感器数据处理

传感器数据处理链，其中原始传感器数据被转换成表征机器人环境状态的各种特征，主要的感知传感器是激光雷达，彩色立体相机和红外立体相机，这些传感器产生的数据流将被单独处理，并将其产生的特征组合到世界模型中。

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/sensor_process.png)

### Stereo Correlation and Filtering

应用了一系列前置和后置滤波器，以帮助增加立体图像的视差密度并消除虚假的深度数据：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/stereo_pipeline.png)

### Ground and Object Segmentation

为了安全地导航，将传感器点云分为地面和目标对象：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ground_object_segment.png)

- 立体图像地面分割
- 点云地面分割

## Terrain Classification

自主导航需要地形分类，以提供对机器人环境内物体和表面类型的态势感知：

- 决策树
  - ![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/decision_tree.png)
- 深度卷积网络
  - ![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/cnn_classification.png)

### Roughness Detection

诸如岩石和树干的小物体可能会对平台造成穿越危险。但是，它们的大小可能小于世界模型的分辨率（0.2 m），这受内存和计算限制，但是，立体摄像机具有足够的原始分辨率，可以检测到这些微小的危害。

### Negative Obstacle Detection

负面障碍是一类特别危险的障碍，可能对平台造成灾难性的破坏。在此项目中，我们着重研究了在尘土上遇到的两个最危险的负面障碍：陡峭的侧倾和突然的跌落。

## SENSOR FUSION

世界模型负责维护机器人环境的模型，并创建代价地图图，以供路径规划系统使用。除了处理机载传感器数据外，世界模型还维护了多个先验信息数据库，可用于增强和补充观测数据。尽管先验数据库非常重要，但先验数据的使用不在本文讨论范围之内。相反，我们仅专注于机载传感器数据的融合，这对于在无法获得先验数据的远征环境中操作尤为重要。

###  Multi-sensor Integration

Stereo + LIDAR

### Voxel Grid

OctoMap

## EXPERIMENTS AND RESULTS

感知管道的所有组件都设置了伪实时要求，以10 Hz 的频率运行，以最大程度地减少各种时滞并保持对动态环境的响应。

## 阅读总结

环境感知管道：

1. 标定
2. 数据处理：滤波、分割、分类
3. 数据融合构建 World Model（八叉树地图）

