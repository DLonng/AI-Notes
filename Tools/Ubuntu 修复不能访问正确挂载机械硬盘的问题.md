给主机加装了一个机械硬盘，但是到 Ubuntu 内不能正确挂载：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ubable_access_yingpan.png)

找了下方法，发现直接用 `ntfsfix` 修复即可：

```shell
sudo apt-get install ntfs-3g
```

修复要挂载的硬盘分区，比如修复 `/dev/sdb1`，这个分区位置从上面的报错信息中查看：

```
sudo ntfsfix /dev/sdb1
```

执行完即可正确挂载了。

参考链接：

- https://askubuntu.com/questions/462381/cant-mount-ntfs-drive-the-disk-contains-an-unclean-file-system
- https://blog.csdn.net/lanxuezaipiao/article/details/25137351