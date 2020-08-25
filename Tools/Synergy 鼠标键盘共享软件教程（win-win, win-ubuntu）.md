之前在朋友圈发了一个我的 Window 和 Ubuntu 共享一套鼠标键盘的视频，挺多朋友回复也有这个需求，今天登龙就跟大家分享一下 2 台电脑共享键鼠的教程，非常简单。

## 一、准备工作

在开始前你需要如下的准备：

- 两台电脑连接到同一个路由器或交换机，或者连接到同一个 Wifi，保证能够相互 ping 通
- 两台电脑上分别安装同版本的 synergy 共享键鼠软件

最好使用路由器或者交换机，如果网络不稳定使用 Wifi 共享会卡顿，关注公众号「登龙」回复「共享键鼠」即可获取我正在用的 synergy 免费软件：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/v1_8_8.png)

## 二、Windows - Windows 共享键鼠

### 2.1 安装软件

使用 2 台 Windows 的朋友配置共享键鼠很容易，先分别在两台电脑上安装 synergy 软件，注意电脑的位数，如果是 64 位的就安装 x64.msi，如果是 32 位则安装 x86.msi，安装过程一路 next 下去即可：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/install_synergy.png)

### 2.2 相互 Ping 测试

把两台 Window 同时用网线连接到路由器或者交换机，或者连接同一个 Wifi，我用的交换机：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/connect.png)

分别在每台 Windows 上按下「Windows + R」，输入 cmd 并确定：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/cmd.png)

然后在黑窗口里面输入「ipconfig」查看当前电脑有线网络的 ip 地址，如果你用 wifi 要查看无线网卡的 ip：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/win_ip1.png)

第二台 Windows 也同样，然后记下 2 个电脑的 ip，比如我的是：

- Windows1: 192.168.0.109
- Windows2: 192.168.0.210

然后在一台电脑上的黑窗口中输入 「ping + ip」命令来测试与第二台电脑是否存在网络连接，因为这个 synergy 软件是需要通过网络传输数据的：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ping_ip.png)

同样在第二台电脑也同样 ping 一下第一台电脑，如果相互都能够接收到回复，则说明两台电脑能够通过网络通信了，如果不能 ping 通，可以拉到文末先看下解决方法。

### 2.3 配置 synergy 共享键鼠

主要前面 2 步成功了，ping 也能收到回复，这个软件的配置其实很容易的。首先你要确定你哪台电脑为服务器，哪台为客户端：

- 服务器端：连接鼠标键盘的电脑
- 客户端：通过 synergy 软件连接服务器端电脑的鼠标和键盘

也就是说你连接鼠标和键盘的哪台电脑为服务器，另外一台为客户端，这个要搞清楚，然后开始分别配置。

#### 2.3.1 配置服务器端 synergy 软件

启动服务端（连接键鼠）synergy 软件，选择 Server 模式，并设置服务器：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/server_config1.png)

设置服务器是为了确定另一台客户端电脑要放的位置和主机名，直接从右上角拖动一个电脑下来，选择左或者右放下去：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/server_config2_2.png)

然后双击刚拖下来的电脑，重命名为客户端的主机名：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/server_config3_3.png)

客户端的屏幕名在客户端软件上有，打开客户端 synergy 即可看到，我的叫 DL-Win10：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/server_config6.png)

直接填上，然后点击 OK！：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/server_config7.png)

这样我们服务端 synergy 就配置完成了，其实就是拖下来一个表示客户端的电脑，并设置下屏幕名而已。

#### 2.3.2 配置客户端 synergy 软件

客户端很简单，把 Client 打钩，直接把服务端电脑的 ip 填进去，点击右下角启动，即可完成与服务端键鼠的连接了：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/server_config5.png)

如果启动成功，你的鼠标应该可以在两个屏幕间切换了，如果不能连接的话，基本都是网络没有 ping 通造成的，参考文末解决方法。

## 三、Windows - Ubuntu 共享键鼠

不使用 Ubuntu 的同学可以跳过这节了，我工作时使用两台电脑，左边 Windows 右边 Ubuntu，如下：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/me.jpg)

所以非常需要共享鼠标键盘，我的 Ubuntu 台式机连接键鼠作为服务器，左边笔记本 Windows10 作为客户端，配置方法和前面的完全一样，只是在 Ubuntu 上安装软件和查看 ip 会有点不同。

### 3.1 Ubuntu 安装 Synergy

```shell
sudo dpkg -i synergy-v1.8.8-stable-Linux-x86_64.deb
```

如果缺少依赖，使用 apt 解决：

```shell
sudo apt install -f
```

再重新安装：

```shell
sudo dpkg -i synergy-v1.8.8-stable-Linux-x86_64.deb
```

#### 3.2 Ubuntu 查看 ip

在 Ubuntu 上查看 ip 用的是 「ifconfig」：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ifconfig_ubuntu.png)

## 四、Windows - Mac 共享键鼠

除了在 Mac 上安装软件方法不同外，其他的完全一样。

## 五、常见问题

### 5.1 Ping 不通怎么办？

把 2 台电脑的防火墙都关闭。

### 5.2 能 Ping 通但是不能共享

检查客户端的屏幕名是否填对。

### 5.3 屏幕位置错位了

如果客户端在左边，但是鼠标向右边滑出去才能到客户端，说明你设置服务器的时候客户端电脑的位置放反了，试试调下方向。

### 5.4 其他问题

待补充，也可以直接微信私聊我。