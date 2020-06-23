# 创建语义融合 ROS 节点

## 一、创建 ROS 包

```shell
cd catkin_ws/src
```

```shell
catkin_create_pkg [pkg_name] std_msgs rospy roscpp
```

## 二、修改包信息

## 2.1 package.xml

package.xml 文件是描述功能包清单的文件，包括功能包的名称、版本号、作者信息、许可信息、编译依赖和运行依赖等。

- `<buildtool_depend>autoware_build_flags</buildtool_depend>` 定义功能包依赖的其他构建工具
- `<build_depend></build_depend>` 定义功能包编译时所依赖的功能包
- `<exec_depend></exec_depend>` 定义了功能包可执行程序运行时所依赖的功能包

```xml
<buildtool_depend>autoware_build_flags</buildtool_depend>
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

例子：

```xml
<?xml version="1.0"?>
<package format="2">
    <name>joint_pixel_pointcloud</name>
    <version>1.9.1</version>
    <description>joint_pixel_pointcloud_node</description>
    <maintainer email="sun_miao@zju.eduy.cn"> sunm </maintainer>
    <license>sunm </license>

    <buildtool_depend>catkin</buildtool_depend>
    <buildtool_depend>autoware_build_flags</buildtool_depend>

    <build_depend>cv_bridge</build_depend>
    <build_depend>image_transport</build_depend>
    <build_depend>pcl_conversions</build_depend>
    <build_depend>pcl_ros</build_depend>
    <build_depend>roscpp</build_depend>
    <build_depend>qtbase5-dev</build_depend>
    <build_depend>tf</build_depend>
    <build_depend>yunle_sensor_msgs</build_depend>

    <exec_depend>cv_bridge</exec_depend>
    <exec_depend>image_transport</exec_depend>
    <exec_depend>pcl_conversions</exec_depend>
    <exec_depend>pcl_ros</exec_depend>
    <exec_depend>roscpp</exec_depend>
    <exec_depend>libqt5-core</exec_depend>
    <exec_depend>tf</exec_depend>
    <exec_depend>yunle_sensor_msgs</exec_depend>

    <build_depend>qtbase5-dev</build_depend>
    <exec_depend>libqt5-core</exec_depend>

</package>
```

## 2.2 CMakeLists.txt







## 问题

1. 如何确定创建功能包时的依赖项？