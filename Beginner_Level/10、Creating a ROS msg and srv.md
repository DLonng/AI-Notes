## 10、Creating a ROS msg and srv
### 1、msg 和 srv 文件
msg 文件是一个简单的文本文件，用来使用不同的语言生成 messages 源代码，并且 msg 文件经常以 `Header header` 为开头的单独一行。

srv 文件用来描述一个 service 服务，它包含 2 部分：一个请求，一个响应，两者以 `---` 分隔：
```
int64 A
int64 B
---
int64 Sum
```
例如 A 和 B 是请求，Sum 是响应。
### 2、创建 msg 
1. 创建 msg/xxx.msg 文件
2. 修改 package.xml 文件
3. 修改 CMakeList.txt 文件

具体过程查看官网。

### 3、创建 srv
不用手动创建，拷贝已有的 srv 文件，然后修改 `package.xml` 和 `CMakeList.txt` 文件。

具体过程查看官网。

### 4、重新 make 项目
进入 catkin work space：
```
catkin_make install
```
即可在 devel 文件夹下的 `include`，`lib`，`share` 查看 C++，Python，Lisp 的 msg 编译输出文件，而对于 Python 和 Lisp 语言的 srv 文件则会存放在单独的 srv 文件夹下。

具体路径查看官网教程。