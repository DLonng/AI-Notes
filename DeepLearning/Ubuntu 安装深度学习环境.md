### 一、Python 3.6

- https://www.pythonf.cn/read/104542

源码编译安装：

https://blog.csdn.net/JohinieLi/article/details/79574666

自带版本和 3.6 版本相互切换：

https://blog.csdn.net/lzzyok/article/details/77413968

使用 pip3 install 出错：

```shell
Exception:
Traceback (most recent call last):
  File "/usr/local/lib/python3.6/site-packages/pip/basecommand.py", line 215, in main
    status = self.run(options, args)
  File "/usr/local/lib/python3.6/site-packages/pip/commands/install.py", line 272, in run
    with self._build_session(options) as session:
  File "/usr/local/lib/python3.6/site-packages/pip/basecommand.py", line 72, in _build_session
    insecure_hosts=options.trusted_hosts,
  File "/usr/local/lib/python3.6/site-packages/pip/download.py", line 329, in __init__
    self.headers["User-Agent"] = user_agent()
  File "/usr/local/lib/python3.6/site-packages/pip/download.py", line 93, in user_agent
    from pip._vendor import distro
  File "/usr/local/lib/python3.6/site-packages/pip/_vendor/distro.py", line 1050, in <module>
    _distro = LinuxDistribution()
  File "/usr/local/lib/python3.6/site-packages/pip/_vendor/distro.py", line 594, in __init__
    if include_lsb else {}
  File "/usr/local/lib/python3.6/site-packages/pip/_vendor/distro.py", line 931, in _get_lsb_release_info
    raise subprocess.CalledProcessError(code, cmd, stdout, stderr)
subprocess.CalledProcessError: Command 'lsb_release -a' returned non-zero exit status 1.
```

复制一份 `lsb_release.py` 即可：

```shell
# ls /usr/lib/python3/dist-packages/lsb_release.py
/usr/lib/python3/dist-packages/lsb_release.py

# ls /usr/local/lib/python3.6
...

# sudo cp /usr/lib/python3/dist-packages/lsb_release.py /usr/local/lib/python3.6/
```

因为默认的 pip3 版本不高：

```shell
pip3 -V
```

所以更新 pip3：

```shell
sudo apt-get install python3-pip
sudo pip3 install --upgrade pip
```

参考博客：https://www.pythonf.cn/read/104542

## 二、Pytorch_0.4.1

官网下载：

https://pytorch.org/get-started/previous-versions/

选择版本 torch-0.4.1-cp36-cp36m-linux_x86_64：

https://download.pytorch.org/whl/cu90/torch_stable.html

测试安装是否成功，发现 Python3 找不到 numpy，安装即可：

```
pip3 install numpy
```

测试成功！

参考博客：https://blog.csdn.net/majinlei121/article/details/83281117

## 三、CUDA 9.0

cuda-9.0：

- https://developer.nvidia.com/cuda-90-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1604&target_type=runfilelocal

我把 cuda-9.0 装在机械硬盘上，测试 `nvcc -V` 的时候发现没有权限，这是因为挂载硬盘的时候没有给文件可执行权限，导致 bin 目录下的文件不能执行，改变下挂载命令，去掉文件权限掩码即可：

```shell
UUID=EA9CC11E9CC0E663      /home/dlonng/data3    ntfs    defaults,utf8,uid=1000,gid=1000,dmask=022,fmask=133     0       0	
```

去掉 fmask：

```shell
UUID=EA9CC11E9CC0E663      /home/dlonng/data3    ntfs    defaults,utf8,uid=1000,gid=1000,dmask=022     0       0	
```

如果你没有配置挂载权限，那应该不会出现这个问题。

## 四、CUDNN 7.1

cudnn-7.1.4：

- https://developer.nvidia.com/rdp/cudnn-archive

安装：

```shell
# 解压
tar -zxvf cudnn-9.0-linux-x64-v7.1.tgz 

# 拷贝到 cuda-9.0 安装目录下
sudo cp cuda/include/cudnn.h /home/dlonng/data3/cuda-9.0/include
sudo cp cuda/lib64/libcudnn* /home/dlonng/data3/cuda-9.0/lib64 -d

# 确保所有用户可读
sudo chmod a+r /home/dlonng/data3/cuda-9.0/include/cudnn.h
sudo chmod a+r /home/dlonng/data3/cuda-9.0/lib64/libcudnn*
```

测试是否安装成功：

```shell
nvcc -V
```

```shell
cat /usr/local/cuda/include/cudnn.h | grep CUDNN_MAJOR -A 2
```

参考博客：

- https://www.cnblogs.com/pertor/p/8733010.html
- http://www.twistedwg.com/2018/06/15/cuda9_cudnn7.html

## visdom

pip3 install 出错：

```shell
Retrying (Retry (total=4, connect=None, read=None, redirect=None, status=None)) after connection broken by 'ConnectTimeoutError
```

替换 pip3 安装源，比如替换成阿里云的：

```shell
pip install xxx -i http://mirrors.aliyun.com/pypi/simple/
```

常用的 pip 源：

```shell
阿里云 http://mirrors.aliyun.com/pypi/simple/

中国科技大学 https://pypi.mirrors.ustc.edu.cn/simple/

豆瓣 (douban) http://pypi.douban.com/simple/

清华大学 https://pypi.tuna.tsinghua.edu.cn/simple/

中国科学技术大学 http://pypi.mirrors.ustc.edu.cn/simple/
```

安装 visdom：

```shell
pip3 install visdom -i https://pypi.tuna.tsinghua.edu.cn/simple/
```

测试：

```shell
$ python3
Python 3.6.4 (default, Jul 29 2020, 09:14:59) 
[GCC 5.4.0 20160609] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import visdom
>>> quit() 
```

## torchsummary

```shell
pip3 install torchsummary -i https://pypi.tuna.tsinghua.edu.cn/simple/
```

## 设置 Python 环境

默认的 requirements.txt 前 2 行少了个等号会提示错误：

```shell
Defaulting to user installation because normal site-packages is not writeable
ERROR: Invalid requirement: 'torch=0.4.1' (from line 1 of requirements.txt)
Hint: = is not a valid operator. Did you mean == ?
```

```shell
torch=0.4.1
torchvision=0.2.1
torchsummary==1.5.1
visdom==0.1.8.4
```

添加等号：

```shell
torch==0.4.1
torchvision==0.2.1
torchsummary==1.5.1
visdom==0.1.8.4
```

使用国内的源来安装不会出错：

```shell
pip3 install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple/
```

## 下载数据集

