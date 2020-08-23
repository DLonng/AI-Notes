#include "c_queue.h"

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

void test_new() {
  ArrayQueue *queue = queue_new();
  assert(empty(queue) == 1);
  free(queue);
}



void test_enqueue() {
  ArrayQueue *queue = queue_new();
  enqueue(queue, 1);
  enqueue(queue, 2);
  enqueue(queue, 3);
  enqueue(queue, 4);
  enqueue(queue, 5);

  print_queue(queue);
  assert(full(queue) == 1);
  free(queue);
}

void test_dequeue() {
  ArrayQueue *queue = queue_new();
  for (int i = 0; i < 5; i++)
    enqueue(queue, i + 1);

  print_queue(queue);
  assert(full(queue) == 1);
  
  assert(dequeue(queue) == 1);
  assert(dequeue(queue) == 2);
  assert(dequeue(queue) == 3);
  assert(dequeue(queue) == 4);
  assert(dequeue(queue) == 5);
  
  assert(empty(queue) == 1);
  
  print_queue(queue);
  free(queue);

}

void test_empty() {
  ArrayQueue *queue = queue_new();
  assert(empty(queue) == 1);
  free(queue);
}

void test_full() {
  ArrayQueue *queue = queue_new();
  for (int i = 0; i < kMaxSize; i++) 
    enqueue(queue, i + 1);
  assert(full(queue) == 1);
  free(queue);
}

void run_all_test() {
  //test_new();
  //test_enqueue();
  //test_full();
  test_dequeue();
}

int main() {
  run_all_test();
  return 0;
}


