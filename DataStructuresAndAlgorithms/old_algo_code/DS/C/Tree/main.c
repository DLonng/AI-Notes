#include "bst_tree.h"

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

void test_insert() {
	BSTNode *root = NULL;
	root = bst_insert(root, 5);
	root = bst_insert(root, 4);
	root = bst_insert(root, 6);
	root = bst_insert(root, 3);
	
	bst_pre_print(root);
	putchar('\n');
	
	bst_in_print(root);
	putchar('\n');
	
	bst_post_print(root);
	putchar('\n');

	bst_delete(root);
}

void test_count() {
	BSTNode *root = NULL;
	root = bst_insert(root, 5);
	root = bst_insert(root, 4);
	root = bst_insert(root, 6);
	root = bst_insert(root, 3);

	assert(bst_get_node_count(root) == 4);	
	bst_delete(root);
}


void test_is_in() {
	BSTNode *root = NULL;
	root = bst_insert(root, 5);
	root = bst_insert(root, 4);
	root = bst_insert(root, 6);
	root = bst_insert(root, 3);

	assert(bst_is_in(root, 4) == 1);	
	assert(bst_is_in(root, 1) == 0);	
	bst_delete(root);
}



void test_height() {
	BSTNode *root = NULL;
	root = bst_insert(root, 5);
	root = bst_insert(root, 4);
	root = bst_insert(root, 6);
	root = bst_insert(root, 3);

	assert(bst_get_height(root) == 3);	
	bst_delete(root);
}


void test_get_min_max() {
	BSTNode *root = NULL;
	root = bst_insert(root, 5);
	root = bst_insert(root, 4);
	root = bst_insert(root, 6);
	root = bst_insert(root, 3);

	assert(bst_get_min(root) == 3);	
	assert(bst_get_max(root) == 6);	
	bst_delete(root);
}


void test_is() {
	BSTNode *root = NULL;
	root = bst_insert(root, 5);
	root = bst_insert(root, 4);
	root = bst_insert(root, 6);
	root = bst_insert(root, 3);

	assert(bst_is(root) == 1);	
	bst_delete(root);
}


void test_delete_value() {
	BSTNode *root = NULL;
	root = bst_insert(root, 5);
	root = bst_insert(root, 4);
	root = bst_insert(root, 6);
	root = bst_insert(root, 3);

	bst_delete_value(root, 3);	
	assert(bst_is_in(root, 3) == 0);	

	bst_delete(root);
}


void test_successor() {
	BSTNode *root = NULL;
	root = bst_insert(root, 5);
	root = bst_insert(root, 4);
	root = bst_insert(root, 6);
	root = bst_insert(root, 3);

	assert(4 == bst_get_successor(root, 3));

	bst_delete(root);
}




void run_all_test() {
//	test_insert();
//	test_count();
//	test_is_in();
//	test_height();
//	test_get_min_max();
//	test_is();
//	test_delete_value();
	test_successor();
}

int main(void) {
	run_all_test();

	return 0;
}
