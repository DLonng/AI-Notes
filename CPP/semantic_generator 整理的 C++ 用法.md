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