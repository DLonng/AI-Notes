我工作中频繁使用 sublime，目前的版本有点低，看起来很丑，所以简单地升级下，记录下步骤：

### 1. 卸载旧版本

```shell
sudo apt remove sublime-text
```

### 2. 安装新版本

```shell
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
```

```shell
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
```

```shell
sudo apt update
```

```shell
sudo apt install sublime-text
```

搞定！

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/sublime-text3-update.png)