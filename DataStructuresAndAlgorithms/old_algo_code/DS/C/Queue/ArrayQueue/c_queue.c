#include "c_queue.h"

#include <stdlib.h>
#include <string.h>


ArrayQueue *queue_new() {
  ArrayQueue *queue = (ArrayQueue *)malloc(sizeof(ArrayQueue));
  queue->head = 0;
  queue->tail = 0;
  memset(queue->data, 0, sizeof(queue->data));
  return queue;
}

void print_queue(ArrayQueue *queue) {
  for (int i = queue->head; i != queue->tail; i = (i + 1) % kMaxPos) {
    printf("%d ", queue->data[i]); 
  }

  putchar('\n');
}


void check_address(ArrayQueue *queue) {
  if (NULL == queue) {
    printf("Malloc memory failed.\n");
	exit(EXIT_FAILURE);
  }
}


void enqueue(ArrayQueue *queue, int item) {
  if (full(queue)) {
    printf("Unable insert to full queue.\n");
	exit(EXIT_FAILURE);
  }

  queue->data[queue->tail] = item;
  queue->tail = (queue->tail + 1) % kMaxPos;
}


int dequeue(ArrayQueue *queue) {
  if (empty(queue)) {
    printf("Unable pop value from empty queue.\n");
	exit(EXIT_FAILURE);
  }
  
  int value = queue->data[queue->head];
  queue->data[queue->head] = 0;
  queue->head = (queue->head + 1) % kMaxPos;
  return value;
}


int empty(ArrayQueue *queue) {
  return queue->head == queue->tail;
}


int full(ArrayQueue *queue) {
  return ((queue->tail + 1) % kMaxPos) == queue->head;
} 



