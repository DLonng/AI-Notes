#include "bst_tree.h"
#include <queue>
#include <climits>
#include <stack>

/**
 * @synopsis get a new BSTNode 
 *
 * @param value: node value
 *
 * @return new BSTNode  
 */
BSTNode *GetNewNode(int value) {
  BSTNode *node = new BSTNode;
  node->data = value;
  node->left = nullptr;
  node->right = nullptr;
  return node;
}


/**
 * @synopsis  Insert a new node
 *
 * @param node
 * @param value
 *
 * @return root node  
 */
BSTNode *Insert(BSTNode *node, int value) {
  if (nullptr == node) {
    node = GetNewNode(value);
	return node;
  } else {
    if (value < node->data)
      node->left = Insert(node->left, value);
    else if (value > node->data)
      node->right = Insert(node->right, value);
    else
		;// do nothing if equal
  }
  return node;
}


/**
 * @synopsis  Search a node
 *
 * @param node
 * @param value
 *
 * @return true or false  
 */
bool Search(BSTNode *node, int value) {
  if (nullptr == node)
    return false;
  if (value < node->data)
    return Search(node->left, value);
  else if (value > node->data)
    return Search(node->right, value);
  else 
    return true;
}


/**
 * @synopsis  GetMinNode 
 *
 * @param node
 *
 * @return   
 */
BSTNode *GetMinNode(BSTNode *node) {
  if (nullptr == node)
    return nullptr;
  if (nullptr == node->left)
    return node;
  return GetMinNode(node->left);
}



/**
 * @synopsis  GetMinData 
 *
 * @param node
 *
 * @return if don't have min data return -1, otherwise return min data  
 */
int GetMinData(BSTNode *node) {
  BSTNode *min_node = GetMinNode(node);
  if (min_node != nullptr)
    return min_node->data;
  else 
    return -1;
}


/**
 * @synopsis  GetMax 
 *
 * @param node
 *
 * @return max data or -1  
 */
int GetMax(BSTNode *node) {
  if (nullptr == node)
    return -1;
  if (nullptr == node->right)
    return node->data;
  return GetMax(node->right);
}




/**
 * @synopsis  DeleteTree 
 *
 * @param node
 */
void DeleteTree(BSTNode *node) {
  if (nullptr == node)
    return;
  if (node->left != nullptr)
    DeleteTree(node->left);
  if (node->right != nullptr)
    DeleteTree(node->right);
  delete node;
}




/**
 * @synopsis  IsBinarySearchTree 
 *
 * @param node
 *
 * @return   
 */
bool IsBinarySearchTree(BSTNode* node) {
	return IsBetween(node, INT_MIN, INT_MAX);
}

/**
 * @synopsis  IsBetween 
 *
 * @param node
 * @param min INT_MIN
 * @param max INT_MAX
 *
 * @return true or false  
 */
bool IsBetween(BSTNode *node, int min, int max) {
	if (nullptr == node)
		return true;
	if (node->data > min && node->data < max && 
			IsBetween(node->left, INT_MIN, INT_MAX) && 
			IsBetween(node->right, INT_MIN, INT_MAX))
		return true;
	else
		return false;
}


/**
 * @synopsis  DeleteValue 
 *
 * @param node
 * @param value
 *
 * @return   
 */
BSTNode *DeleteValue(BSTNode *node, int value) {
	// 1. 判断 node 是否为 nullptr
	if (nullptr == node)
		return nullptr;
	
	// 2. 递归左右子树查找要删除的节点
	if (value < node->data) {
		node->left = DeleteValue(node->left, value);
	} else if (value > node->data) {
		node->right = DeleteValue(node->right, value);
	} else { 
		// 3. 两个子节点都为 nullptr，直接删除当前节点 node 即可
		if ((node->left == nullptr) && (node->right == nullptr)) {
			delete node;
			node = nullptr;
		// 4. 左子树为 nullptr，将右子树作为替换节点
		} else if (nullptr == node->left) {
			BSTNode *tmp = node;
			node = node->right;
			delete tmp;
		// 5. 右子树为 nullptr，将左子树作为替换节点
		} else if (nullptr == node->right) {
			BSTNode *tmp = node;
			node = node->left;
			delete tmp;
		// 6. 两个子节点都不为 nullptr，选择右子树中的最小节点做为替换节点
		} else {
			// 查找右子树的最小节点
			BSTNode *tmp = GetMinNode(node->right);
			// 将节点数据赋值给当前节点
			node->data = tmp->data;
			// 删除右子树的最小节点
			node->right = DeleteValue(node->right, tmp->data);
		}
	}

	return node;
}


/**
 * @synopsis  GetSuccessor 
 *
 * @param node
 * @param value
 *
 * @return   
 */
BSTNode *GetSuccessor(BSTNode *node, int value) {
	// 1. 判断 node 是否为 nullptr
	if (nullptr == node)
		return nullptr;
	// 2. 找到 data == value 的目标节点
	BSTNode *target_node = nullptr;
	if (value < node->data) 
		target_node = node->left;
	else if (value > node->data)
		target_node = node->right;
	else // Has find ok
		;
	// 3. 如果该节点有右子树，就返回右子树中的最小节点
	if (target_node->right != nullptr) {
		return GetMinNode(target_node->right);
	} else {
		// 4. 没有右子树，就在左子树中查找刚好比 value 大的节点
		BSTNode *successor = nullptr;
		BSTNode *ancestor = node;
		while (ancestor != nullptr) {
			if (value < ancestor->data) {
				successor = ancestor;
				ancestor = ancestor->left;
			} else {
				ancestor = ancestor->right;
			}
		}

		// 5. 返回找到的节点
		return successor;
	}
}


/**
 * @synopsis 面试题 - DFS 递归前序遍历 
 *
 * @param node
 */
void PreOrderTraverse(BSTNode *node) {
  if (nullptr == node)
    return;
  // 1. 输出 data 
  std::cout << node->data << " ";
  // 2. 递归左子树
  PreOrderTraverse(node->left);
  // 3. 递归右子树
  PreOrderTraverse(node->right);
}

/**
 * @synopsis  InOrderTraverse 面试题 - DFS 递归中序遍历
 *
 * @param node
 */
void InOrderTraverse(BSTNode *node) {
	if (nullptr == node)
		return;
	// 1. 递归左子树
	InOrderTraverse(node->left);
	// 2. 输出 data
	std::cout << node->data << " ";
	// 3. 递归右子树
	InOrderTraverse(node->right);
}


/**
 * @synopsis  PostOrderTraverse 面试题 - DFS 递归后序遍历
 *
 * @param node
 */
void PostOrderTraverse(BSTNode *node) {
	if (nullptr == node)
		return;
	// 1. 递归左子树
	PostOrderTraverse(node->left);
	// 2. 递归右子树
	PostOrderTraverse(node->right);
	// 3. 输出 data
	std::cout << node->data << " ";
}

/**
 * @synopsis  PreOrderTraverse2 面试题 - DFS 非递归前序遍历
 *
 * @param node
 */
void PreOrderTraverse2(BSTNode *node) {
	if (nullptr == node)
		return;
	// 1. 创建节点栈
	std::stack<BSTNode *> s;
	BSTNode *t = nullptr;
	s.push(node);
	
	while (!s.empty()) {
		t = s.top();
		s.pop();
		// 2. 输出栈顶元素
		std::cout << t->data << " ";
		// 3. 压入右子树
		if (t->right != nullptr)
			s.push(t->right);
		// 4. 压如左子树
		if (t->left != nullptr)
			s.push(t->left);
	}
}

/**
 * @synopsis  InOrderTraverse2 面试题 - DFS 非递归中序遍历
 *
 * @param node
 */
void InOrderTraverse2(BSTNode *node) {
	if (nullptr == node)
		return;
	// 1. 创建节点栈
	std::stack<BSTNode *> s;
	BSTNode *p = node;
	while (p != nullptr || !s.empty()) {
		// 2. 递归左子树
		if (p != nullptr) {
			s.push(p);
			p = p->left;
		} else {
			p = s.top();
			s.pop();
			// 3. 输出 data
			std::cout << p->data << " ";
			// 4. 递归右子树
			p = p->right;
		}
	}
}

/**
 * @synopsis  PostOrderTraverse2 面试题 - DFS 非递归后序遍历
 *
 * @param node
 *
 * 解析：后序便利会有些复杂
 * 要保证根结点在左孩子和右孩子访问之后才能访问，因此对于任一结点 P，先将其入栈。
 * 如果 P 不存在左孩子和右孩子，则可以直接访问它，
 * 如果 P 存在左孩子或者右孩子，但是其左孩子和右孩子都已被访问过了，则同样可以直接访问该结点。
 * 若非上述两种情况，则将 P 的右孩子和左孩子依次入栈，这样就保证了每次取栈顶元素的时候，
 * 左孩子在右孩子前面被访问，左孩子和右孩子都在根结点前面被访问
 */
void PostOrderTraverse2(BSTNode *node) {
	// 1. 创建节点栈
	std::stack<BSTNode *> s;
	BSTNode *cur = nullptr;
	BSTNode *pre = nullptr;
	s.push(node);

	while (!s.empty()) {
		cur = s.top();
		// 2. 如果当前没有孩子节点或者（右）孩子节点已经被访问过了
		if ((nullptr == cur->left && nullptr == cur->right) || (pre != nullptr && (/* pre == cur->left || */pre == cur->right))) {
			// 4. 输出 data
			std::cout << cur->data << " ";
			s.pop();
			// 5. 保存上一次输出的节点位置
			pre = cur;
		} else {
			// 3. 压入左右子树
			if (cur->right != nullptr)
				s.push(cur->right);
			if (cur->left != nullptr)
				s.push(cur->left);
		}
	}
}



/**
 * @synopsis  面试题 - BFS 宽度优先遍历（层序遍历） 
 *
 * @param node
 */
void PrintBFS(BSTNode *node) {
  // 1. 创建遍历队列
  std::queue<BSTNode *> node_queue;
  BSTNode* current = nullptr;
  node_queue.push(node);
  
  while (!node_queue.empty()) {
    // 2. 弹出队列头部元素
    current = node_queue.front();
	node_queue.pop();

	if (current != nullptr) {
      std::cout << current->data << " ";
      // 3. 将左孩子推入队列
	  if (current->left != nullptr) 
	    node_queue.push(current->left);
	  // 4. 将右孩子推入队列
	  if (current->right != nullptr)
        node_queue.push(current->right);
	}
  }
}


/**
 * @synopsis  GetHeight 面试题 - 求二叉树的深度 
 *
 * @param node
 *
 * @return tree height  
 */
int GetHeight(BSTNode *node) {
/*
  if (nullptr == node)
    return 0;
  return std::max(GetHeight(node->left), GetHeight(node->right)) + 1;
*/
	if (nullptr == node)
		return 0;
	int nLeft = GetHeight(node->left);
	int nRight = GetHeight(node->right);
	return (nLeft > nRight) ? (nLeft + 1) : (nRight + 1);
}


bool HasSubTree(BSTNode *pRootA, BSTNode *pRootB) {
	bool res = false;
	// 1. 递归遍历树 A 和 B
	if (pRootA != nullptr && pRootB != nullptr) {
		// 2. 如果有一个节点相同，则调用判断函数
		if (pRootA->data == pRootB->data)
			res = IsTreeAHasTreeB(pRootA, pRootB);
		// 3. 如果不相同，递归左右子树，但不递归树 B
		if (!res)
			res = HasSubTree(pRootA->left, pRootB);
		if (!res)
			res = HasSubTree(pRootA->right, pRootB);
	}
	return res;
}


bool IsTreeAHasTreeB(BSTNode *pRootA, BSTNode *pRootB) {
	// 1. 判断树是否为空
	if (nullptr == pRootA)
		return false;
	// 2. 空树是任何树的子树
	if (nullptr == pRootB)
		return true;
	// 3. 有一个节点值不同即返回 false
	if (pRootA->data != pRootB->data)
		return false;
	// 4. 递归判断所有叶子节点
	return IsTreeAHasTreeB(pRootA->left, pRootB->left) &&
		IsTreeAHasTreeB(pRootA->right, pRootB->right);
}



/**
 * @synopsis  GetLeafNum 面试题 - 得到叶子节点的数量
 *
 * @param root
 *
 * @return   
 */
int GetLeafNum(BSTNode *root) {
	// 1. 判断 root 是否为空
	if (nullptr == root)
		return 0;
	// 2. 左右孩子都是空的是叶子节点
	if (nullptr == root->left && nullptr == root->right)
		return 1;
	// 3. 递归计算左右子树
	return GetLeafNum(root->left) + GetLeafNum(root->right);
}


// 面试题 - 输出一个节点的父节点
int GetFatherNode(BSTNode *root, BSTNode *node) {
	// 1. 判断 root 是否为空
	if (nullptr == root)
		return 0;

	// 2. 判断父节点的左节点是否符合条件
	if ((root->left != nullptr) && (node->data == root->left->data)) {
		std::cout << root->data << std::endl;
		return root->data;
	}
	
	// 3. 判断父节点的右节点是否符合条件
	if ((root->right != nullptr) && (node->data == root->right->data)) {
		std::cout << root->data << std::endl;
		return root->data;
	}	

	return GetFatherNode(root->left, node);
	return GetFatherNode(root->right, node);
}

// 面试题 - 得到节点的左兄弟
int GetLeftNode(BSTNode *root, BSTNode *node) {
	if (nullptr == root)
		return 0;
	// 1. 判断当前节点的右孩子是否是 node
	if ((root->right != nullptr) && (node->data == root->right->data)) {
		// 2. 表示当前节点是父节点，然后输出左孩子即可
		if (root->left != nullptr) {
			std::cout << root->left->data << std::endl;
			return root->left->data;
		}
	}

	return GetLeftNode(root->left, node);
	return GetLeftNode(root->right, node);
}



/**
 * @synopsis  GetNodeNum 面试题 - 计算节点的个数 
 *
 * @param root
 *
 * @return   
 */
int GetNodeNum(BSTNode *root) {
	// 1. 返回状态
	if (nullptr == root)
		return 0;
	else // + 1 表示根节点 
		return GetNodeNum(root->left) + GetNodeNum(root->right) + 1;
}




/**
 * @synopsis  GetFullNode 面试题 - 计算满节点的个数 
 *
 * @param root
 *
 * @return   
 *
 * 1. 树为空，满节点个数为 0
 * 2. 只有一个节点，满节点个数为 0
 * 3. 只有左子树，需要进一步判断左子树中是否存在满节点
 * 3. 只有右子树，需要进一步判断右子树中是否存在满节点
 * 5. 如果左右都有，说明是满节点。但是由于它的左子树或者右子树中可能还存在满结点，因 
 *    此满结点个数等于该节点加上该节点的左子树中满结点的个数再加上右子树中满结点的个数
 */
int GetFullNode(BSTNode *root) {
	int nodes = 0;
	if (nullptr == root)
		return 0;
	else if (nullptr == root->left && nullptr == root->right)
		return 0;
	else if (root->left != nullptr && nullptr == root->right)
		nodes = GetFullNode(root->left);
	else if (nullptr == root->left && root->right != nullptr)
		nodes = GetFullNode(root->right);
	else 
		nodes = GetFullNode(root->left) + GetFullNode(root->right) + 1;
	
	return nodes;
/*
 	// 这两种递归的方式有和区别？
	if (nullptr == root)
		return 0;
	else if (nullptr == root->left && nullptr == root->right)
		return 0;
	else if (root->left != nullptr && nullptr == root->right)
		return GetFullNode(root->left);
	else if (nullptr == root->left && root->right != nullptr)
		return GetFullNode(root->right);
	else 
		return GetFullNode(root->left) + GetFullNode(root->right) + 1;
*/

/*
 	// 对于二叉树而言，度为 2 的节点个数等于度为 0（叶子节点） 的节点个数减去 1：n(2) = n(0) - 1
 	return GetLeafNum(root) > 0 ? GetLeafNum(root) - 1 : 0;
 */
}





























