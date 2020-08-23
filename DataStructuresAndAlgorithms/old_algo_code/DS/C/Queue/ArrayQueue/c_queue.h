#ifndef ARRAY_QUEUE_H_
#define ARRAY_QUEUE_H_

#include <stdio.h>

#define kMaxSize 5 

#define kMaxPos 6

typedef struct ArrayQueue {
  int data[kMaxPos];
  int head;
  int tail;
} ArrayQueue;

ArrayQueue *queue_new();

void print_queue(ArrayQueue *queue);

void check_address(ArrayQueue *queue);

void enqueue(ArrayQueue *queue, int item);

int empty(ArrayQueue *queue);

int full(ArrayQueue *queue);

int dequeue(ArrayQueue *queue);

#endif //ARRAY_QUEUE_H_

