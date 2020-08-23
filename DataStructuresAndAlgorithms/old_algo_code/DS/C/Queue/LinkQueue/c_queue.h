#ifndef QUEUE_LINK_H_
#define QUEUE_LINK_H_


#include <stdio.h>
#include <stdlib.h>


typedef struct Node {
  int data;
  struct Node *next;
} node_t;


typedef struct CQueue {
  node_t *head;
  node_t *tail;
} CQueue;


CQueue *cqueue_new();

void check_address(void *address);

void print_queue(CQueue *queue);

void enqueue(CQueue *queue, int item);

int dequeue(CQueue *queue);

int empty(CQueue *queue);

#endif //QUEUE_LINK_H_

