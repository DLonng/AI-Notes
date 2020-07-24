昨天把一个工作空间的 `lidar_camera_fusion` 包拷贝到另外一个工作空间编译，但是名字没有改变，导致 `source `后系统存在 2 个 `lidar_camera_fusion` 包，使用的时候总是错误地启动另外一个。

解决方式：把其中一个包的配置信息改变即可，要改变的文件有

- `CMakeLists.txt`
- `package.xml`

在 `CMakeLists.txt` 中更改项目名和输出的可执行文件名：

```cmake
project(更换项目名称)

add_executable(更换可执行文件名称
  src/lidar_camera_fusion.cpp
  src/main.cpp)
```

在 package.xml 中更改包的基本信息，这一步不是必须的，与编译无关，但是最好也改下，毕竟要与这个包本身信息同步，以后别人看或者自己回头修改才不会产生疑惑：

```xml
<package format = "2">
    <name> 要修改的包名 </name>
	...
</package>
```

然后重新编译即可：

```shell
catkin_make
```

以后复制包的时候记得更改一下包的名称，防止系统中 2 个工作空间存在相同的包，导致 roslaunch 启动找到另外一个包启动了。