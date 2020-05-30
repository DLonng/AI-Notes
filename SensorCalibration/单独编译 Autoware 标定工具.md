## 单独编译 Autoware 标定工具

## 一、编译过程

- https://blog.csdn.net/qq_42615787/article/details/102481314

安装顺序：

1. SWIG-4.0.1
2. nlopt-2.4.2
3. 在 ros 工作空间编译 calibration_camera_lidar

注意：nlopt 安装 2.4.2 版本，最新版本最报 [nlopt_get_errmsg 的引用错误](https://github.com/prusa3d/PrusaSlicer/issues/4259)！

## 二、使用问题

Load 相机内参只能使用 `yml` 格式，但是自己标定的相机内参是 `yaml` 格式的文件！