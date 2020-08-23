#include "max_heap.h"


const int kQueueCapacity = 1000;

void maxheap_insert(MaxHeap *heap, int value) {
	if (heap->size == heap->capacity) {
		printf("Can't add more items.\n");
		exit(EXIT_FAILURE);
	}

	heap->size++;
	heap->elements[heap->size] = value;
	
	maxheap_sift_up(heap, heap->size);
}

void maxheap_sift_up(MaxHeap *heap, int heap_size) {
	int temp = 0;
	int parent = 0;

	while (heap_size > 1) {
		parent = heap_size / 2;
		
		if (heap->elements[parent] < heap->elements[heap_size]) {
			temp = heap->elements[heap_size];
			heap->elements[heap_size] = heap->elements[parent];
			heap->elements[parent] = temp;
		}

		heap_size = parent;
	}
}



int maxheap_get_max(MaxHeap *heap) {
	return heap->elements[1];
}

int maxheap_get_size(MaxHeap *heap) {
	return heap->size;
}


int maxheap_is_empty(MaxHeap *heap) {
	return !heap->size;
}

int maxheap_extract_max(MaxHeap *heap) {
	int max = heap->elements[1];
	heap->elements[1] = heap->elements[heap->size];
	heap->size--;
	
	maxheap_sift_down(heap, 1);
	return max;
}


int maxheap_sift_down(MaxHeap *heap, int index) {
	int i = index;
	int swap_index = 0;
	int temp = 0;

	while (i * 2 <= heap->size) {
		int left = 2 * i;
		int right = 2 * i + 1;
		bool has_left  = (left  <= heap->size ? 1 : 0);
		bool has_right = (right <= heap->size ? 1 : 0);

		if (has_left && has_right) {
			int left_value = heap->elements[left];
			int right_value = heap->elements[right];
			
			if (left_value > right_value)
				swap_index = left_value;
			else
				swap_index = right_value;
		} else if (has_left) {
			swap_index = left;
		} else if (has_right) {
			swap_index = right;
		} else {
			break;
		}

		if (heap->elements[swap_index] > heap->elements[i]) {
			temp = heap->elements[i];
			heap->elements[i] = heap->elements[swap_index];
			heap->elements[swap_index] = temp;

			i = swap_index;
		} else {
			break;
		}
	}
}



void maxheap_remove_node(MaxHeap *heap, int index) {
	heap->elements[index] = heap->elements[heap->size];
	heap->size--;

	maxheap_sift_down(heap, index);
}



void maxheap_heapify(int *numbers, int count) {
	for (int i = count / 2 - 1; i >= 0; i--)
		maxheap_percolate_down(numbers, count, i);
}


void maxheap_percolate_down(int *numbers, int count, int index) {
	int i = index;
	int swap_index = 0;
	int temp = 0;

	while ((i * 2) + 1 < count) {
		int left = 2 * i + 1;
		int right = 2 * i + 2;
		bool has_left  = (left  < count ? 1 : 0);
		bool has_right = (right < count ? 1 : 0);

		if (has_left && has_right) {
			int left_value = numbers[left];
			int right_value = numbers[right];
			if (left_value > right_value) 
				swap_index = left;
			else 
				swap_index = right;
		} else if (has_left) {
			swap_index = left;
		} else if (has_right) {
			swap_index = right;
		} else {
			break;
		}

		if (numbers[swap_index] > numbers[i]) {
			temp = numbers[i];
			numbers[i] = numbers[swap_index];
			numbers[swap_index] = temp;

			i = swap_index;
		} else {
			break;
		}
	}
}




void maxheap_sort(int *numbers, int count) {
	int temp = 0;
	maxheap_heapify(numbers, count);

	for (int i = count - 1; i > 0; i--) {
		temp = numbers[0];
		numbers[0] = numbers[i];
		numbers[i] = temp;

		maxheap_percolate_down(numbers, i, 0);
	}
}


void maxheap_print(MaxHeap *heap) {
	for (int i = 1; i <= heap->size; i++) 
		printf("%4d |", heap->elements[i]);
	putchar('\n');
}










































