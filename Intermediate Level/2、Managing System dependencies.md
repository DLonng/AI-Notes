# Managing System dependencies

## 1、rosdep
rosdep 工具可以在系统缺少依赖时，帮助安装依赖文件。

## 2、如何查看依赖
在功能包的 package.xml 文件中定义了包的依赖文件：
```
cat package.xml
```
例如：
```
<package>

...
...
  <build_depend>message_generation</build_depend>
  <build_depend>libqt4-dev</build_depend>
  <build_depend>qt4-qmake</build_depend>
  <build_depend>rosconsole</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>roscpp_serialization</build_depend>
  <build_depend>roslib</build_depend>
  <build_depend>rostime</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>std_srvs</build_depend>
</package>
```

## 3、如何安装依赖
当某个功能包缺少依赖时，使用下面的命令来安装依赖：
```shell
rosdep install [package_name]
```
如果以上命名错误，是因为没有初始化 rosdep，执行：
```shell
sudo rosdep init
rosdep update
```

如果你的依赖没问题，则提示：
```shell
All required rosdeps installed successfully
```

如果缺少依赖，rosdep 会自动安装：
```shell
#!/usr/bin/bash

set -o errexit
set -o verbose


if [ ! -f /opt/ros/lib/libboost_date_time-gcc42-mt*-1_37.a ] ; then
  mkdir -p ~/ros/ros-deps
  cd ~/ros/ros-deps
  wget --tries=10 http://pr.willowgarage.com/downloads/boost_1_37_0.tar.gz
  tar xzf boost_1_37_0.tar.gz
  cd boost_1_37_0
  ./configure --prefix=/opt/ros
  make
  sudo make install
fi


if [ ! -f /opt/ros/lib/liblog4cxx.so.10 ] ; then
  mkdir -p ~/ros/ros-deps
  cd ~/ros/ros-deps
  wget --tries=10 http://pr.willowgarage.com/downloads/apache-log4cxx-0.10.0-wg_patched.tar.gz
  tar xzf apache-log4cxx-0.10.0-wg_patched.tar.gz
  cd apache-log4cxx-0.10.0
  ./configure --prefix=/opt/ros
  make
  sudo make install
fi
```
安装完成，自动退出。