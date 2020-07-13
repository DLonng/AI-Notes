### 点云结构体内存对齐

```cpp
union {
    // data[0] = x, data[1] = y, data[2] = z
    // data[3] 用来保证内存对齐
    float data[4];
    struct {
        float x;
        float y;
        float z;
    };
};
```

为何需要保证内存对齐？

### 友元类 friend class

```cpp
template <class SEMANTICS> class SemanticsOcTree;

class SemanticsOcTreeNode : public ColorOcTreeNode {
    
};
```

为何要使用 friend class？

### Doxygen 代码注释风格

文件注释：

```

```

类注释：

```

```

函数注释：

```cpp
/**
  * @brief 函数简要说明
  * @details 函数细节说明
  * @param[in] 参数名 参数描述
  * @param[out] 参数名 参数描述
  * @return void or other
  * @note 注意事项
  * @todo 需要去实现的功能
  * @author 作者
  * @date 日期
  */
```

参考链接：

-  https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/comments/
- https://blog.csdn.net/guyue35/article/details/46775211
- [Doxygen](https://www.cnblogs.com/silencehuan/p/11169084.html)

### C++ 重载与重写

参考博客：

- [C++, 重载，重写，重定义的总结](https://www.cnblogs.com/tanky_woo/archive/2012/02/08/2343203.html)