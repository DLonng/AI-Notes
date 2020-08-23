#include "bst_tree.h"

#include <stdio.h>
#include <stdlib.h>

BSTNode *bst_insert(BSTNode *node, const int value) {
	if (NULL == node) {
		BSTNode *new_node = (BSTNode *)malloc(sizeof(BSTNode));
		bst_check_address(new_node);
		new_node->data = value;
		new_node->left = NULL;
		new_node->right = NULL;
		node = new_node;
	}

	if (value < node->data) {
		node->left = bst_insert(node->left, value);
	} else if (value > node->data) {
		node->right = bst_insert(node->right, value);
	} else { 
		// equal don`t thing.
	}

	return node;
}

void bst_check_address(BSTNode *node) {
	if (NULL == node) {
		printf("Malloc memory fail.\n");
		exit(-1);
	}
}

void bst_pre_print(BSTNode *node) {
	if (NULL == node) {
		return;
	}

	printf("%d\n", node->data);
	bst_pre_print(node->left);
	bst_pre_print(node->right);
}


void bst_in_print(BSTNode *node) {
	if (NULL == node) {
		return;
	}

	bst_in_print(node->left);
	printf("%d\n", node->data);
	bst_in_print(node->right);
}



void bst_post_print(BSTNode *node) {
	if (NULL == node) {
		return;
	}

	bst_post_print(node->left);
	printf("%d\n", node->data);
	bst_post_print(node->right);
}



int bst_get_node_count(BSTNode *node) {
	if (NULL == node) 
		return 0;
	return 1 + bst_get_node_count(node->left) + bst_get_node_count(node->right);
}


void bst_delete(BSTNode *node) {
	if (NULL == node) 
		return ;
	bst_delete(node->left);
	bst_delete(node->right);
	free(node);
}

int bst_is_in(BSTNode *node, const int value) {
	if (NULL == node)
		return 0;
	if (value < node->data)
		return bst_is_in(node->left, value);
	else if (value > node->data) 
		return bst_is_in(node->right, value);
	else
		return 1;
}



int bst_get_height(BSTNode *node) {
	if (NULL == node)
		return 0;
	return 1 + max_num(bst_get_height(node->left), bst_get_height(node->right));
}


int max_num(const int a, const int b) {
	return a >= b ? a : b;
}


int bst_get_min(BSTNode *node) {
	if (NULL == node)
		return 0;
	BSTNode *current = node;
	while (current->left != NULL)
		current = current->left;
	return current->data;
}

int bst_get_max(BSTNode *node) {
	if (NULL == node)
		return 0;
	BSTNode *current = node;
	while (current->right != NULL)
		current = current->right;
	return current->data;
}


int bst_is(BSTNode *node) {
	if (NULL == node)
		return 1;
	return bst_is_between(node, INT_MIN, INT_MAX);
}


int bst_is_between(BSTNode *node, int min, int max) {
	if (NULL == node)
		return 1;
	return node->data > min && node->data < max &&
		bst_is_between(node->left, min, node->data) &&
		bst_is_between(node->right, node->data, max);
}


BSTNode *bst_delete_value(BSTNode *node, int value) {
	if (NULL == node)
		return NULL;
	if (value < node->data) {
		node->left = bst_delete_value(node->left, value);
	} else if (value > node->data) {
		node->right = bst_delete_value(node->right, value);
	} else {
		if ((NULL == node->left) && (NULL == node->right)) {
			free(node);
			node = NULL;
		} else if (NULL == node->left) {
			BSTNode *tmp = node;
			node = node->left;
			free(tmp);
			tmp = NULL;
		} else if (NULL == node->right) {
			BSTNode *tmp = node;
			node = node->right;
			free(tmp);
			tmp = NULL;
		} else {
			int right_min = bst_get_min(node->right);
			node->data = right_min;
			node->right = bst_delete_value(node->right, right_min);
		}
	}

	return node;
}



int bst_get_successor(BSTNode *node, int value) {
	if (NULL == node)
		return -1;
	
	BSTNode *target = node;
	
	while (target->data != value) {
		if (value < target->data)
			target = target->left;
		else if (value > target->data)
			target = target->right;
	}

	if (target->right != NULL) {
		return bst_get_min(target->right);
	} else {
		BSTNode *successor = NULL;
		BSTNode *ancestor = node;
		while (ancestor != NULL) {
			if (value < ancestor->data) {
				successor = ancestor;
				ancestor = ancestor->left;
			} else {
				ancestor = ancestor->right;
			}
		}
		
		return successor->data;
	}
}
