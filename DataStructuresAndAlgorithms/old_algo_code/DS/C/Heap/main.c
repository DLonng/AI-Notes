#include "max_heap.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>




static void init_heap(MaxHeap *queue) {
	maxheap_insert(queue, 35);
	maxheap_insert(queue, 33);
	maxheap_insert(queue, 42);
	maxheap_insert(queue, 10);
	maxheap_insert(queue, 14);
	maxheap_insert(queue, 19);
	maxheap_insert(queue, 27);
	maxheap_insert(queue, 44);
	maxheap_insert(queue, 26);
	maxheap_insert(queue, 31);
	
}



static void test_size() {
	MaxHeap queue;
	queue.size = 0;
	queue.capacity = 1000;
	
	init_heap(&queue);

	assert(maxheap_get_size(&queue) == 10);
}


static void test_max() {
	MaxHeap queue;
	queue.size = 0;
	queue.capacity = 1000;
	
	init_heap(&queue);

	assert(maxheap_get_max(&queue) == 44);
}

static void test_extract_max() {
	MaxHeap queue;
	queue.size = 0;
	queue.capacity = 1000;
	
	init_heap(&queue);

	assert(maxheap_extract_max(&queue) == 44);
}


static void test_print() {
	MaxHeap queue;
	queue.size = 0;
	queue.capacity = 1000;
	
	init_heap(&queue);

	printf("test_print:\n");
	maxheap_print(&queue);
}


static void test_remove() {
	MaxHeap queue;
	queue.size = 0;
	queue.capacity = 1000;
	
	init_heap(&queue);
	maxheap_remove_node(&queue);
	maxheap_print(&queue);
}

static void test_all() {
	//test_size();
	//test_extract_max();
	//test_max();

	//test_print();
}

int main(void) {
	test_all();	
	return 0;
}


