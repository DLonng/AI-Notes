#ifndef BST_TREE_H
#define BST_TREE_H

#include <iostream>

struct BSTNode {
  int data;
  BSTNode *left;
  BSTNode *right;
};

// Return a new node for our BST
BSTNode *GetNewNode(int value);

// Insert a new node
BSTNode *Insert(BSTNode *node, int value);

// Search a node that 'node->data == value'
bool Search(BSTNode *node, int value);

// Get min node
BSTNode *GetMinNode(BSTNode *node);

// Get min data 
int GetMinData(BSTNode *node);

// Get max node
int GetMax(BSTNode *node);

// Get tree height
int GetHeight(BSTNode *node);

// Delete tree
void DeleteTree(BSTNode *node);

// Return true if tree is a binary search tree
bool IsBinarySearchTree(BSTNode* node);

// Return true if all items in the given tree are between the given values
bool IsBetween(BSTNode *node, int min, int max);

// Delete a specific node from the tree
BSTNode *DeleteValue(BSTNode *node, int value);

// Return the in-order successor of the given value
BSTNode *GetSuccessor(BSTNode *node, int value);

// 面试题 - 递归前序遍历
void PreOrderTraverse(BSTNode *node);

// 面试题 - 递归中序遍历
void InOrderTraverse(BSTNode *node);

// 面试题 - 递归后序遍历
void PostOrderTraverse(BSTNode *node);

// 面试题 - 非递归前序遍历
void PreOrderTraverse2(BSTNode *node);

// 面试题 - 非递归中序遍历
void InOrderTraverse2(BSTNode *node);

// 面试题 - 非递归后序遍历
void PostOrderTraverse2(BSTNode *node);

// 面试题 - 宽度优先遍历（层序遍历） 
void BFS(BSTNode *node);

// 面试题 - 判断树 B 是否是树 A 的子树
bool HasSubTree(BSTNode *pRootA, BSTNode *pRootB);
bool IsTreeAHasTreeB(BSTNode *pRootA, BSTNode *pRootB); 

// 面试题 - 得到叶子节点的数量
int GetLeafNum(BSTNode *root);

// 面试题 - 得到一个节点的父节点
int GetFatherNode(BSTNode *root, BSTNode *node);

// 面试题 - 得到节点的左兄弟
int GetLeftNode(BSTNode *root, BSTNode *node);

// 面试题 - 得到二叉树的节点数
int GetNodeNum(BSTNode *root);

// 面试题 - 得到度为 2 的节点
int GetFullNode(BSTNode *root);






#endif // BST_TREE_H
