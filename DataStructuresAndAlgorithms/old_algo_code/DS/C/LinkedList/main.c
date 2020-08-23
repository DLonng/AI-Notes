#include "c_linklist.h"

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

void test_size() {
  node_t *head = (node_t *)malloc(sizeof(node_t));
  
  head->data = 1;
  head->next = NULL;

  assert(size(head) == 1);

}

void test_empty() {
  node_t *head1 = NULL;
  assert(empty(head1) == 1);

  node_t *head2 = (node_t *)malloc(sizeof(node_t));
  assert(empty(head2) == 0);
}


void test_push() {
  node_t *head = NULL;
  push_back(&head, 1);
  push_back(&head, 2);
  push_back(&head, 3);
  assert(size(head) == 3);
}

void test_print() {
  node_t *head = NULL;
  push_back(&head, 1);
  push_back(&head, 2);
  push_back(&head, 3);
  print_list(head);
  putchar('\n');

}

void test_value_at() {
  node_t *head = NULL;
  push_back(&head, 1);
  push_back(&head, 2);
  push_back(&head, 3);
  assert(value_at(head, 0) == 1);
  assert(value_at(head, 1) == 2);
  assert(value_at(head, 2) == 3);
}


void test_push_front() {
  node_t *head = NULL;
  push_back(&head, 1);
  push_back(&head, 2);
  push_back(&head, 3);
  push_front(&head, 11);
  assert(value_at(head, 0) == 11);
  assert(value_at(head, 1) == 1);
  assert(size(head) == 4);
 
}

void test_pop_front() {
  node_t *head = NULL;
  push_back(&head, 1);
  push_back(&head, 2);
  push_back(&head, 3);
  push_front(&head, 11);
  
  assert(size(head) == 4);
  assert(pop_front(&head) == 11);
  assert(size(head) == 3);
}


void test_pop_back() {
  node_t *head = NULL;
  push_back(&head, 1);
  push_back(&head, 2);
  push_back(&head, 3);
  push_front(&head, 11);
  
  assert(size(head) == 4);
  assert(pop_back(&head) == 3);
  assert(size(head) == 3);
}

void test_front() {
  node_t *head = NULL;
  push_back(&head, 1);
  push_back(&head, 2);
  push_back(&head, 3);
  
  assert(front(head) == 1);
 
}

void test_back() {
  node_t *head = NULL;
  push_back(&head, 1);
  push_back(&head, 2);
  push_back(&head, 3);
  
  assert(back(head) == 3);
 

}

void test_insert() {
  node_t *head = NULL;
  push_back(&head, 1);
  push_back(&head, 2);
  push_back(&head, 3);
  
  insert(&head, 0, 11);
  insert(&head, 2, 22);
  insert(&head, 5, 33);
  assert(value_at(head, 0) == 11);
  assert(value_at(head, 2) == 22);
  assert(value_at(head, 5) == 33);
  print_list(head);
}


void test_erase() {
  node_t *head = NULL;
  push_back(&head, 1);
  push_back(&head, 2);
  push_back(&head, 3);
  push_back(&head, 4);
  push_back(&head, 5);
  
  erase(&head, 0);
  erase(&head, 1);
  erase(&head, 2);
  
  assert(value_at(head, 0) == 2);
  assert(value_at(head, 1) == 4);
  print_list(head);
}

void test_value_of_from_end() {
  node_t *head = NULL;
  push_back(&head, 1);
  push_back(&head, 2);
  push_back(&head, 3);
  push_back(&head, 4);
  push_back(&head, 5);
 

  assert(value_n_from_end(head, 2) == 4);

}


void test_reverse() {
  node_t *head = NULL;
  push_back(&head, 1);
  push_back(&head, 2);
  push_back(&head, 3);
  push_back(&head, 4);
  push_back(&head, 5);

  print_list(head);
  reverse(&head);
  print_list(head);
}


void test_remove_value() {
  node_t *head = NULL;
  push_back(&head, 1);
  push_back(&head, 2);
  push_back(&head, 2);
  push_back(&head, 4);
  push_back(&head, 5);

  remove_value(&head, 2);
  remove_value(&head, 4);
  print_list(head);
  putchar('\n');
}




void run_all_test() {
  //test_size();
  //test_empty();
  //test_push();
  //test_print();
  //test_value_at();
  //test_push_front();
  //test_pop_front();
  //test_pop_back();
  //test_front();
  //test_back();
  //test_insert();
  //test_erase();
  //test_value_of_from_end();
  //test_reverse();
  test_remove_value();
}



int main(void) {
  run_all_test();
}


