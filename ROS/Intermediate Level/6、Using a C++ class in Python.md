# Using a C++ class in Python
## 1、创建 C++ 功能包
```shell
catkin_create_pkg python_bindings_tutorial rospy roscpp std_msgs
cd python_bindings_tutorial/include/python_bindings_tutorial
touch add_two_ints.h
rosed python_bindings_tutorial add_two_ints.h
```

未测试通过。