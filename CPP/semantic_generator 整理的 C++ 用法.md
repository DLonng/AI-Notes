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

### 代码注释风格

使用 Doxygen 风格：

```cpp
/**
 * Manipulate log_odds value of a voxel by changing it by log_odds_update (relative).
 * This only works if key is at the lowest octree level
 *
 * @param key OcTreeKey of the NODE that is to be updated
 * @param log_odds_update value to be added (+) to log_odds value of node
 * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
 *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
 * @return pointer to the updated NODE
 */
```

-  https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/comments/
- https://blog.csdn.net/guyue35/article/details/46775211
- [Doxygen](https://www.cnblogs.com/silencehuan/p/11169084.html)

### C++ 重载与重写

参考博客：

- [C++, 重载，重写，重定义的总结](https://www.cnblogs.com/tanky_woo/archive/2012/02/08/2343203.html)