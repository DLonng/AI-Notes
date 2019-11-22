---
title: PCL - Overview
date: 2019-11-22 20:00:00
---
# PCL - Overview
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

## Overview
PCL 库被分为一系列的模块，比较重要的有以下这些：
- Filters
- Features
- Keypoints
- Registration
- KdTree
- Octree
- Segmentation
- Sample Consensus
- Surface
- Range Image
- IO
- Visualization
- Common
- Search

下面分别简单介绍下这些模块的基本作用。

## Filters
在实际数据采集中，由于测量误差，某些数据集会出现大量阴影点，这些阴影点会使得局部点云的估计变得不准确，因此需要用统计分析的方法来过滤掉这些误差数据，这就称为滤波。

比如下面这张图中黑色圆圈中的阴影点就是测量的误差或称为噪声：


















<div  align="center">
<img src="https://dlonng.com/images/xxx/xxx.png"/>
</div>

> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>