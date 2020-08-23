#ifndef BST_TREE_H_
#define BST_TREE_H_

#include <stdio.h>
#include <limits.h>

typedef struct BSTNode {
	int data;
	struct BSTNode *left;
	struct BSTNode *right;
} BSTNode;

BSTNode *bst_insert(BSTNode *node, const int value);

void bst_check_address(BSTNode *node);

void bst_pre_print(BSTNode *node);

void bst_in_print(BSTNode *node);

void bst_post_print(BSTNode *node);

int bst_get_node_count(BSTNode *node);

void bst_delete(BSTNode *node);

int bst_is_in(BSTNode *node, const int value);

int bst_get_height(BSTNode *node);

int max_num(const int a, const int b);

int bst_get_min(BSTNode *node);

int bst_get_max(BSTNode *node);

int bst_is(BSTNode *node);

int bst_is_between(BSTNode *node, int min, int max);

BSTNode *bst_delete_value(BSTNode *node, int value);

int bst_get_successor(BSTNode *node, int value);

#endif // BST_TREE_H_


