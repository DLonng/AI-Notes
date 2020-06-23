

## 一、Octomap 框架

官方文档：[Octomap](http://octomap.github.io/octomap/doc/)



![](http://octomap.github.io/octomap/doc/uml_overview.png)

继承 OccupancyOcTreeBase 即可实现自己的 Octree 类，参考 [OcTreeStamped](http://octomap.github.io/octomap/doc/classoctomap_1_1OcTreeStamped.html) 和 [OcTreeNodeStamped](http://octomap.github.io/octomap/doc/classoctomap_1_1OcTreeNodeStamped.html) 类。

## 主要数据结构

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

一个容器类，用于对内部 key 的寻址，keys 计算从 origin 开始的体素数量作为一个体素的离散地址。

#### 3.1.4 octomap::KeyRay

KeyRay 用于保存单条光束在三维空间中射线跟踪的结果。

#### 3.1.5 octomap::KeySet

把 OcTreeKey 保存在 KeyHash 表中，以进行更加有效率的更新（Hash 表访问速度 O(1)），KeySet 收纳所有光束（也即点云数据）射线跟踪的结果。

## Octree 常用函数

### castRay

从原点 origin 在指定方向 direction 上做一个射线投影，输出第一个非空单元的中心坐标 end：

```cpp
bool octomap::OccupancyOcTreeBase< NODE >::castRay	(	const point3d & 	origin,
    const point3d & 	direction,
    point3d & 	end,
    bool 	ignoreUnknownCells = false,
    double 	maxRange = -1.0 
)		const
```

| [in]  |       origin       | starting coordinate of ray                                   |
| ----- | :----------------: | ------------------------------------------------------------ |
| [in]  |     direction      | A vector pointing in the direction of the raycast (NOT a point in space). Does not need to be normalized. |
| [out] |        end         | returns the center of the last cell on the ray. If the function returns true, it is occupied. |
| [in]  | ignoreUnknownCells | whether unknown cells are ignored (= treated as free). If false (default), the raycast aborts when an unknown cell is hit and returns false. |
| [in]  |      maxRange      | Maximum range after which the raycast is aborted (<= 0: no limit, default) |

如果命中了一个占用的单元格，则返回 true，如果达到最大范围或八叉树界限，或者命中了未知节点，则返回 false。

### computeRayKeys

跟踪一个从原点 origin 到终点（不包括） end 的射线，返回射线光束遍历的所有节点的 OcTreeKey：

```cpp
bool octomap::OcTreeBaseImpl< NODE, AbstractOccupancyOcTree >::computeRayKeys	(	const point3d & 	origin,
const point3d & 	end,
KeyRay & 	ray 
)		const
```

参数 KeyRay，用于保存光线所遍历的所有节点的 Key，但不包括 end。操作成功返回 true，返回 false 通常意味着其中一个坐标超出了OcTree 的范围。

### coordToKey

把一个单独的坐标转换为一个离散的 Key：

```cpp
typedef uint16_t octomap::key_type

key_type octomap::OcTreeBaseImpl< NODE, AbstractOccupancyOcTree >::coordToKey	(	double 	coordinate	)	const
```

把一个坐标转换为一个 3D OcTreeKey：

```cpp
key_type k[3];

// 一个容器类
OcTreeKey (key_type a, key_type b, key_type c)

OcTreeKey octomap::OcTreeBaseImpl< NODE, AbstractOccupancyOcTree >::coordToKey	(	const point3d & 	coord	)	const
```

### keyToCoord

从给定深度的离散键转换为对应于键中心的坐标：

```cpp
double octomap::OcTreeBaseImpl< NODE, AbstractOccupancyOcTree >::keyToCoord	(	key_type 	key,
unsigned 	depth 
)		const
```

### coordToKeyChecked

通过边界检查将 3D 坐标转换为 3D OcTreeKey：

```cpp
bool octomap::OcTreeBaseImpl< NODE, AbstractOccupancyOcTree >::coordToKeyChecked	(	const point3d & 	coord,
OcTreeKey & 	key 
)		const
```

如果点在八叉树内，则为 true（有效），否则为 false。

### enableChangeDetection

插入扫描时跟踪或忽略更改（默认值：忽略）:

```cpp
void octomap::OccupancyOcTreeBase< NODE >::enableChangeDetection	(	bool 	enable	)	
```

### insertPointCloud

集成点云（在全局参考框架中），需要特别注意的是，地图中的每个体素仅更新一次，并且占用的节点优先于空闲的节点：

```cpp
void octomap::OccupancyOcTreeBase< NODE >::insertPointCloud	(	const Pointcloud & 	scan,
    const octomap::point3d & 	sensor_origin,
    double 	maxrange = -1.,
    bool 	lazy_eval = false,
    bool 	discretize = false 
)
```

### insertRay

在起始点和终点间插入射线：

```cpp
bool octomap::OccupancyOcTreeBase< NODE >::insertRay	(	const point3d & 	origin,
const point3d & 	end,
double 	maxrange = -1.0,
bool 	lazy_eval = false 
)	
```

### read

```cpp
AbstractOcTree * octomap::AbstractOcTree::read	(	const std::string & 	filename	)	

AbstractOcTree* tree = AbstractOcTree::read("filename.ot");
OcTree* octree = dynamic_cast<OcTree*>(tree);
```

### search

在给定深度的搜索节点处指定 3d 点（depth = 0：搜索整个树的深度），你需要检查返回的节点是否为 NULL，因为它可以位于未知空间中：

```cpp
NODE* octomap::OcTreeBaseImpl< NODE, AbstractOccupancyOcTree >::search	(	double 	x,
    double 	y,
    double 	z,
    unsigned int 	depth = 0 
)		const
```

### updateInnerOccupancy

更新所有内部节点的占用率以反映其 children 的占用率：

```cpp
void octomap::OccupancyOcTreeBase< NODE >::updateInnerOccupancy	()	
```

### updateNode

```cpp
NODE * octomap::OccupancyOcTreeBase< NODE >::updateNode	(	const OcTreeKey & 	key,
	float 	log_odds_update,
	bool 	lazy_eval = false 
)
```

```cpp
NODE * octomap::OccupancyOcTreeBase< NODE >::updateNode	(	const OcTreeKey & 	key,
bool 	occupied,
bool 	lazy_eval = false 
)	
```



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

插入射线需要起始点和终点，在终点位置添加一个 node（或者已有 node 的概率升高），在起始点到终点之间经过的 node 概率降低（可以认为起始点是观测点，终点是目标点，那么我既然观测到了目标点，说明目标点之前没有遮挡，如果有遮挡，可能是误差造成的，那就需要对这个遮挡点进行降权） - 复制 CSDN

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
