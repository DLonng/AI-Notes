

## 一、Octomap 框架

官方文档：[Octomap](http://octomap.github.io/octomap/doc/)



![](http://octomap.github.io/octomap/doc/uml_overview.png)

主要数据结构和方法：

- octomap::OcTree：octomap 主要地图数据结构，存储 3D 占用栅格地图

  ![](http://octomap.github.io/octomap/doc/classoctomap_1_1OcTree__inherit__graph.png)

- OcTree::insertRay(...)：插入一束射线

- OcTree::insertPointCloud(...)：插入点云（在全局参考框架中）

- OcTree::search(...)，OcTree::castRay(...)：查询功能

- updateInnerOccupancy：更新地图

- updateNode(...)：插入三维空间点到地图

- leaf_iterator，tree_iterator, leaf_bbx_iterator：访问节点的迭代器

- setProHit/setProMiss：这两个函数决定了 inverse sensor model 的 log-odd 概率更新的具体参数

- setClampingThresMax/setClampingThresMin：这两个函数决定了一个体元执行 log-odd 更新的阈值范围。也就是说某一个占据体元的概率值爬升到 0.97（对应的log-odd为3.5）或者空闲体元的概率值下降到 0.12（对应的log-odds为-2）便不再进行 log-odds 更新计算

- setOccupancyThres：这个函数定义了 octomap 判定某一个体元属于占据状态的阈值（isNodeOccupied函数），默认是 0.5，一般情况下我们将其设定为 0.7

- octomap::OcTreeNode::setLogOdds：设置节点的对数占有概率

- writeBinary(...)：存储地图

- bin/octovis：3D 可视化栅格地图工具



## 二、Octomap 基本数据类型

#### 3.1.1 octomap::OcTree

octomap 主要地图数据结构，维护一个 3D 占用栅格地图。

#### 3.1.2 octomap::OcTreeNode

八叉树节点，存储对数占有概率 log-odds。

#### 3.1.3 octomap::OcTreeKey

实现对八叉树节点 OcTreeNode 的查询，即离散的三维体元所处的空间地址。

#### 3.1.4 octomap::KeyRay

KeyRay 用于保存单条光束在三维空间中射线跟踪的结果。

#### 3.1.5 octomap::KeySet

把 OcTreeKey 保存在 KeyHash 表中，以进行更加有效率的更新（Hash 表访问速度 O(1)），KeySet 收纳所有光束（也即点云数据）射线跟踪的结果。



## 三、Octomap 基本编程

### 4.1 建树

```cpp
// 这是最简单的 octomap，没有设置颜色，0.03 是分辨率可以自己更改，单位米
octomap::OcTree* octomap = new octomap::OcTree(0.03);

// 这是彩色 octomap，0.03 是分辨率可以自己更改，单位米
octomap::ColorOcTree* octomap = new octomap::ColorOcTree(0.03);
```

### 4.2 插入点

```cpp
// updateNode 是插入单个点，接受两个参数，第一个就是 octomap::point3d 类型的 3d 点
// 第二个参数默认填 true 表示占用 occupy，false 表示 free
double x = 1;
double y = 2;
double z = 3;
octomap->updateNode(octomap::point3d(x, y, z), true);
```

### 4.3 插入射线

插入射线需要起始点和终点，在终点位置添加一个 node（或者已有node的概率升高），在起始点到终点之间经过的 node概率降低（可以认为起始点是观测点，终点是目标点，那么我既然观测到了目标点，说明目标点之前没有遮挡，如果有遮挡，可能是误差造成的，那就需要对这个遮挡点进行降权） - 复制 CSDN

```cpp
// insertRay 是插入射线，有两个参数，依次是起始点和终点，都是 octomap::point3d 类型的 3d 点
// 例子是以（0，0，0）为起点，（1，2，3）为终点的射线
double x = 1;
double y = 2;
double z = 3;
octomap->insertRay(octomap::point3d(0, 0, 0), octomap::point3d(x, y, z));
```

### 4.4 加上颜色

```cpp
// 给彩色 octomap 设定 RGB 颜色, 对应六个参数分别是目标点 xyz 颜色 RGB
unsigned int R = 216;
unsigned int G = 120;
unsigned int B = 255;
octomap->setNodeColor(x, y, z, R, G, B);
```

### 4.5 清空和剪枝

```cpp
//清空octomap
octomap->clear()
 
//剪枝（原理见高翔老师博客）
octomap->prune()
```

### 4.6 发布 Octomap Msg 主题

```cpp
// 声明 advertise，octomap rviz plugin 默认接受 topic 为 octomap_full 的 message
pub_octomap = n.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);

// 声明 message
// Octomap map_msg; 是不是写错了 ?
octomap_msgs map_msg;

// 设置 header
map_msg.header.frame_id = 'world';
map_msg.header.stamp = ros::Time::now();

// fullMapToMsg 负责转换成 message
if (octomap_msgs::fullMapToMsg(*octomap, map_msg))
    // 转换成功，可以发布了
    pub_octomap.publish(map_msg);
else
    ROS_ERROR("Error serializing OctoMap");
```

### 4.7 旋转插入局部地图

```cpp
pcl::transformPointCloud(cloud, *temp, pose.matrix());
octomap->insertPointCloud(cloud_octo, octomap::point3d(x, y, z));
```







## 四、一些总结

log-odd 与概率值之间可以相互转化，因此在工程实现时 octomap 八叉树节点类 OcTreeNode 存储的数值是 log-odd 数值，并不是概率值。



## 