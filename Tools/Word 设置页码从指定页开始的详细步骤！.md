「设置页码从指定页开始」这个功能一直是很多同学写论文头疼的问题，今天刚好写了个机器学习小论文的作业，这里把我设置页码从第三页正文开始的步骤分享给大家，其实非常简单：

- 先给封面、目录、正文等分 3 节
- 再设置页脚

### 1. 先分节

要想设置页脚从指定页开始，必须要分节，把一个文档分为不同的节，这样页脚就不会一直连续了！比如我要给封面和目录分节，那么我点击封面底部进入编辑状态，然后点击「页面布局 -> 下一页」即可插入分解符。

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/1_fenjie.png)

### 2. 选择是否链接前一节

相同的节可以把当前页的页脚链接到上一页，但是不同的节就不行，因此可以利用分节来从 1 开始设置页码。双击封面页脚区域，编辑页脚：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/2_bianji.png)

因为封面不需要设置页脚，所以直接点击「设计 -> 下一节」：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/3_next.png)

因为目录也不需要设置页码，所以我又点了一次「下一节」，这里的「链接到前一条页眉」可取消也可不取消，因为上一节是封面没有设置页码，因此即使链接也没关系：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/mulu.png)

从目录节点击「下一节」到正文第一页，现在就是正文这一节了，我们从正文第一页开始设置页码，因为要把这一页作为第一页，所以要取消「链接到前一条页眉」：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/4_cancel.png)

### 3. 添加页码

然后我们就可以点击「插入 -> 页码」设置页码格式，这里必须要选择「起始页码从 1 开始」，取消续前节：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/5_set.png)

点击确定后，再点击「页码」插入一个合适的页码即可，我一般用的是「页面底端 -> 页码居中」的风格：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/6_ok.png)

之后的正文因为都是一节，所以页码会自动递增，这样就完成页码从正文第一页开始的设置了！真是麻烦，还是 MarkDown 好用！

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/yingliu_code/yinliu_code.png)