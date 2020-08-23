#include "c_queue.h"

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>


void test_new() {
  CQueue *queue = cqueue_new();
  assert(queue->head == NULL);
  assert(queue->tail == NULL);
  free(queue);
}

void test_enqueue() {
  CQueue *queue = cqueue_new();
  enqueue(queue, 1);
  enqueue(queue, 2);
  enqueue(queue, 3);
  enqueue(queue, 4);

  print_queue(queue);
}

void test_dequeue() {
  CQueue *queue = cqueue_new();
  enqueue(queue, 1);
  enqueue(queue, 2);
  enqueue(queue, 3);
  enqueue(queue, 4);

  print_queue(queue);

  assert(dequeue(queue) == 1);
  assert(dequeue(queue) == 2);
 
  print_queue(queue);
}

void test_empty() {
  CQueue *queue = cqueue_new();
  assert(empty(queue) == 1); 
  enqueue(queue, 1);
  enqueue(queue, 2);
  enqueue(queue, 3);
  assert(empty(queue) == 0); 
}

void run_all_test() {
  //test_new();
  //test_enqueue();
  //test_dequeue();
  test_empty();
}

int main(void) {
  run_all_test();
  return 0;
}
