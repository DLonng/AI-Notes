我用 VSCode 来 Coding，这个编辑器需要自己配置头文件路径，就是自动建立一个 `c_cpp_properties.json` 文件来管理头文件路径，然后需要用哪些库就手动加上即可，方法很简单，如下：

### 1. 生成 c_cpp_properties.json 文件

按「F1」启动指令输入框，输入 `C/C++`，选择第一项 Edit Configuration：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/vscode_c_cpp.png)

然后会自动生成一个 Json 文件：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/vscode_json_file.png)

### 2. 添加头文件路径

我们只需要再红框的 IncludePath 内加上需要的头文件路径即可，比如我的工程：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/my_vscode_json.png)

这里提示下，常用库的头文件常见安装位置如下：

- `/usr/include/`
- `/usr/local/include`

所以这两个基本要加上的，如果你不知道你安装的库的头文件在哪，但是知道关键的头文件名称，则可以用 `locate` 命令来查找：

```shell
locate ros.h | grep include
```

这个命令的意思是查找所有 ros.h 的位置，并且找出路径中带有 include 字段的路径：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/locate_ros.png)

这样就可以找到自己的头文件路径啦！


![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/yingliu_code/yinliu_code.png)