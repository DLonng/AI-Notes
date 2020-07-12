### cv::Size

```c
typedef struct {
    int width;/* 矩形宽 */
    int height; /* 矩形高 */
} CvSize;
```

### cv::Mat

总结一些常用的初始化矩阵的方法：

```cpp
cv::Mat m；

cv::Mat m(int rows, int cols, int type);

cv::Mat m = cv::Mat::zeros(int rows, int cols, int type);

cv::Mat m = cv::Mat::ones(int rows, int cols, int type);

cv::Mat m = cv::Mat::eye(int rows, int cols, int type);
```

参考博客：

- [OpenCV3 cv::Mat 的定义与初始化](https://blog.csdn.net/guduruyu/article/details/66973415)

### ROS Msg

在编写节点的过程中主要用到以下 2 类 ROS Msg：

- [std_msg](http://wiki.ros.org/std_msgs)：用于订阅普通的消息
  - [Float32MultiArray](http://docs.ros.org/api/std_msgs/html/msg/Float32MultiArray.html)
- [sensor_msgs](http://wiki.ros.org/sensor_msgs)：用于常用传感器，相机雷达等的话题订阅回调函数的参数类型
  - [CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
  - [PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
  - [Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)

