这部分是项目 [semantic_slam](https://github.com/jsdd25/semantic_slam) 的语义分割部分，自己对深度学习和 Python 不是太了解，正好借这个语义分割的代码学习一下，以下是阅读过程中不懂或者疑惑的地方。

### 一、Python 基础

补充下 Python 基础：

- https://www.liaoxuefeng.com/wiki/1016959663602400

### 二、查阅的资料

### 2.1 reshape(-1,  1)

把矩阵变为一列：

```python
self.semantic_color_vect[:, 0 : 1] = semantic_color[:, :, 0].reshape(-1, 1)
```

参考博客：

- [numpy.reshape(-1,1)](https://blog.csdn.net/qq_42804678/article/details/99062431?utm_medium=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.nonecase&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.nonecase)

### 2.2 numpy 矩阵

- numpy.zeros：全 0 矩阵

  ```python
  # x 个 y 行 z 列的 3 维矩阵
  np.ones((x,y,z))
  
  # 3 个 2 行 4 列的 3 维矩阵
  np.ones((3, 2, 4))
  
  array([[[ 0.,  0.,  0.,  0.],
          [ 0.,  0.,  0.,  0.]],
  
         [[ 0.,  0.,  0.,  0.],
          [ 0.,  0.,  0.,  0.]],
  
         [[ 0.,  0.,  0.,  0.],
          [ 0.,  0.,  0.,  0.]]])
  ```

  [Numpy 创建 3 维矩阵](https://blog.csdn.net/qq_38290604/article/details/89711494?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.nonecase&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.nonecase)

- numpy.ones：全 1 矩阵

- numpy.shape：读取每个维度的大小

  ```python
  np.shape([[1, 1], [2, 2], [3, 3]])
  # 3 行 2 列
  # shape[0] = 3, shape[1] = 2
  (3L, 2L)
  ```

  [Python numpy 函数：shape 用法](https://blog.csdn.net/qq_28618765/article/details/78081959)



### 2.3 numpy view

Python 副本和视图的区别：

- 副本是一个数据的完整的拷贝，如果我们对副本进行修改，它不会影响到原始数据，物理内存不在同一位置。

- 视图 view 是数据的一个别称或引用，通过该别称或引用亦便可访问、操作原有数据，但原有数据不会产生拷贝。如果我们对视图进行修改，它会影响到原始数据，物理内存在同一位置。
- 切片数据 `[:]` 也会返回「视图」对象。
- `ndarray.view()` 返回一个新的数组对象，并可查看原始数组的相同数据，但新数组的维数更改不会更改原始数据的维数。

```python
self.ros_data[:, 5 : 6] = self.semantic_color_vect.view('<f4')
```

数据类型 `dtype` 中 `<f4` 的小于号是字节序的类型：

- `<` = little-endian (LSB first)
- `>` = big-endian (MSB first)

参考链接：

- https://www.yiibai.com/numpy/numpy_copies_and_views.html
- https://numpy.org/doc/stable/reference/generated/numpy.ndarray.view.html
- https://stackoverflow.com/questions/40589499/what-do-the-signs-in-numpy-dtype-mean

### 2.4 numpy ravel

`ros_data.ravel() `将多维数组转换为 1 维数组：

```python
self.cloud_ros.data = np.getbuffer(self.ros_data.ravel())[:]
```

参考博客：

- [numpy 中的 ravel ()、flatten ()、squeeze () 的用法与区别](https://blog.csdn.net/tymatlab/article/details/79009618)

### 2.5 bgr8

Python 的 Opencv img 与 ROS img 相互转换：

```python
from cv_bridge import CvBridge, CvBridgeError

self.bridge = CvBridge()

# ros img -> cv img
color_img = self.bridge.imgmsg_to_cv2(color_img_ros, "bgr8")
```

```python
from cv_bridge import CvBridge, CvBridgeError

self.bridge = CvBridge()

# cv img -> ros img
semantic_color_msg = self.bridge.cv2_to_imgmsg(semantic_color, encoding="bgr8")
```

CvBridge 将根据需要选择进行颜色或像素深度转换， 第二个参数为以下的编码方式：

- `mono8`: `CV_8UC1`, grayscale image
- `mono16`: `CV_16UC1`, 16-bit grayscale image
- `bgr8`: `CV_8UC3`, color image with blue-green-red color order
- `rgb8`: `CV_8UC3`, color image with red-green-blue color order
- `bgra8`: `CV_8UC4`, BGR color image with an alpha channel
- `rgba8`: `CV_8UC4`, RGB color image with an alpha channel

参考链接：

- https://blog.csdn.net/weixin_40863346/article/details/80430251

### 2.6 PointCloud2.Fields

在这个项目中需要再 PointCloud2 点云的 Fields 字段中 append 语义字段，我查了下完整的 PointCloud2 类型定义如下：

```cpp
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes 每个点的字节长度
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points
```

一个用法：

```python
self.cloud_ros = PointCloud2()
self.cloud_ros.header.frame_id = frame_id
# 无序点云高度为 1，该属性用来判断点云是否有序
self.cloud_ros.height = 1
self.cloud_ros.width = width * height

self.cloud_ros.fields.append(PointField(
                             name = "x",
                             offset = 0,
                             datatype = PointField.FLOAT32, count = 1))
...
self.cloud_ros.fields.append(PointField(
                            name = "semantic_color",
                            offset = 20,
                            datatype = PointField.FLOAT32, count = 1))
self.cloud_ros.fields.append(PointField(
                            name = "confidence",
                            offset = 24,
                            datatype = PointField.FLOAT32, count = 1))
```

参考链接：

- [sensor_msgs/PointCloud2 Documentation - ros.org](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)

### 2.7 squeeze & unsqueeze



参考博客：

- https://blog.csdn.net/xiexu911/article/details/80820028

### 2.8 torch.topk

```python
pred_confidences, pred_labels  = torch.topk(input = class_probs, k = 3, dim = 1, largest = True, sorted = True)
```



参考链接：

- https://pytorch.org/docs/master/generated/torch.topk.html

### 2.9 pytorch tensor

参考博客：

- https://www.jianshu.com/p/495c7f1e9cfe

### 2.10 softmax

对 n 维输入张量运用 `softmax` 函数，将张量的每个元素缩放到（0,1）区间且和为 1。

参考链接：

- https://pytorch.org/docs/master/generated/torch.nn.Softmax.html#torch.nn.Softmax
- https://pytorch.org/docs/master/nn.functional.html
- https://www.cnblogs.com/wanghui-garcia/p/10675588.html
- https://zhuanlan.zhihu.com/p/108344765

### 2.11 sensor_msg::PointCloud

```cpp
# This message holds a collection of 3d points, plus optional additional
# information about each point.

# Time of sensor data acquisition, coordinate frame ID.
Header header

# Array of 3d points. Each Point32 should be interpreted as a 3d point
# in the frame given in the header.
geometry_msgs/Point32[] points

# Each channel should have the same number of elements as points array,
# and the data in each channel should correspond 1:1 with each point.
# Channel names in common practice are listed in ChannelFloat32.msg.
ChannelFloat32[] channels
```

参考链接：

- http://wiki.ros.org/sensor_msgs
- http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
- http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html
- http://docs.ros.org/api/sensor_msgs/html/msg/PointField.html

### 2.12 计算机字节序



参考链接：

- [https://zh.wikipedia.org/wiki/%E5%AD%97%E8%8A%82%E5%BA%8F#%E5%A4%A7%E7%AB%AF%E5%BA%8F](https://zh.wikipedia.org/wiki/字节序#大端序)



### 2.13 PCL_ADD_RGB 宏的定义



参考链接：

- http://docs.ros.org/hydro/api/pcl/html/structpcl_1_1__PointXYZRGB.html#a14c3aa49bc0ced8ef51dc2a89b616af1
- http://docs.ros.org/hydro/api/pcl/html/point__types_8hpp_source.html



### 2.14 sensor_msg::Image



参考链接：

- http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html