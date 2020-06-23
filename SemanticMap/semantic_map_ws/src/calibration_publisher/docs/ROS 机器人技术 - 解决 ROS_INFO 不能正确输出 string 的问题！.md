## ROS 机器人技术 - 解决 ROS_INFO 不能正确输出 std::string 的问题！

## 一、输出「？？」

项目调试一个节点，打印 ROS 信息时发现设置的节点名称都是问号：

```cpp
ROS_INFO("[%s]: camera_extrinsic_mat", kNodeName);
```

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ros_info_problem.png)

看了下代码发现是自己把节点名称设置为 const，但是没有正确初始化 `const` 变量，导致输出了「？?」，C++ 语法都忘记了，太菜了，以下是第一种初始化 const 的方法：

```cpp
// 1. 先在类中定义
private:
	const std::string kNodeName;

// 2. 构造函数初始化列表后初始化
ClassName(): kNodeName("node_name")
```

第二种方法是加上 static 关键字：

```cpp
// 1. 在类中定义静态 const 变量
private:
	static const std::string kNodeName;
	
// 2. 在类外进行 const 初始化
const std::string ClassName::kNodeName = "node_name";
```

我用的第二种方式，因为这样的 const 看起来比较直观，在构造函数初始化列表中赋值不容易被看到，以下是我修改后的代码：

```cpp
// 1. lidar_camera_fusion.h
private:
	static const std::string kNodeName;

// 2. lidar_camera_fusion.cpp
const std::string LidarCameraFusion::kNodeName = "lidar_camera_fusion";

// 3. ROS_INFO
ROS_INFO("[%s]: camera_extrinsic_mat", kNodeName);
```

总结下 const 变量在类中的用法：

- 构造函数参数初始化列表中初始化 const 变量
- 将 const 变量声明为 static 类型，然后在类的外部初始化

## 二、输出乱码

正确初始化 const 变量后，发现 INFO 又输出乱码：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ros_info_problem2.png)

找了下资料发现是因为 `ROS_INFO` 不能直接输出 `std::string`，需要转成 `c_str`：

```cpp
// 3. ROS_INFO
ROS_INFO("[%s]: camera_extrinsic_mat", kNodeName.c_str());
```

第二个乱码的原因是因为把 `%s` 错写成大写的 `%S`了，改回来即可：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ros_info_problem3.png)

```cpp
// 3. 错写成大写的 %S 了
ROS_INFO("[%S]: camera_extrinsic_mat", kNodeName.c_str());
```

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/yingliu_code/yinliu_code.png)