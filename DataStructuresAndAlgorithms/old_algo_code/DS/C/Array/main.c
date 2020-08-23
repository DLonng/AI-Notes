#include "c_array.h"

#include <stdio.h>
#include <stdlib.h>



void test_resize() {
  CArray *array = carray_new(2);

  int old_capacity = carray_capacity(array);
  assert(old_capacity == 16);

  for (int i = 0; i < 18; i++) {
    carray_push(array, i + 1);
  }
  
  assert(carray_capacity(array) == 32);

  for (int j = 0; j < 15; j++) {
    carray_pop(array);
  }

  assert(carray_capacity(array) == 16);

  carray_destroy(array);
}



void test_init_size() {
  CArray* array = carray_new(2);
  assert(carray_size(array) == 0);

  carray_destroy(array);
}

void test_push() {
  CArray* array = carray_new(2);
  carray_push(array, 1);
  carray_push(array, 11);
  carray_push(array, 111);
  carray_push(array, 1111);
  
  assert(array->size == 4);
  
  carray_destroy(array);
}


void test_empty() {
  CArray* array = carray_new(2);
  assert(carray_is_empty(array) == 1);
  
  carray_push(array, 1);
 
  assert(carray_is_empty(array) == 0);
  
  carray_destroy(array);
}

void test_at() {
  CArray* array = carray_new(2);
  
  carray_push(array, 1);
  carray_push(array, 2);
  carray_push(array, 3);
 
  assert(carray_at(array, 0) == 1);
  assert(carray_at(array, 1) == 2);
  assert(carray_at(array, 2) == 3);
  
  carray_destroy(array);
}

void test_insert() {
  CArray* array = carray_new(2);
  
  carray_push(array, 1);
  carray_push(array, 2);
  carray_push(array, 3);


  carray_insert(array, 1, 22);
  assert(carray_at(array, 1) == 22);
  assert(carray_at(array, 2) == 2);
  
  carray_destroy(array);

}

void test_prepend() {
  CArray* array = carray_new(2);
  
  carray_push(array, 1);
  carray_push(array, 2);
  carray_push(array, 3);


  carray_prepend(array, 11);
  assert(carray_at(array, 0) == 11);
  assert(carray_at(array, 1) == 1);
  
  carray_destroy(array);

}


void test_pop() {
  CArray* array = carray_new(2);
  
  carray_push(array, 1);
  carray_push(array, 2);
  carray_push(array, 3);

  assert(carray_size(array)== 3);

  assert(carray_pop(array) == 3);
  assert(carray_pop(array) == 2);
  assert(carray_pop(array) == 1);
  assert(carray_size(array)== 0);
  
  carray_destroy(array);
}



void test_delete() {
  CArray* array = carray_new(2);
  
  carray_push(array, 1);
  carray_push(array, 2);
  carray_push(array, 3);

  assert(carray_size(array)== 3);
  
  carray_delete(array, 1);
  
  assert(carray_at(array, 1) == 3);
  assert(carray_size(array)== 2);
  
  carray_destroy(array);
}


void test_remove() {
  CArray* array = carray_new(2);
  
  carray_push(array, 1);
  carray_push(array, 2);
  carray_push(array, 3);
  carray_push(array, 3);
  carray_push(array, 3);
  carray_push(array, 4);

  
  carray_remove(array, 3);
  
  assert(carray_find(array, 3)== -1);
  
  carray_destroy(array);
}




void test_find() {
  CArray* array = carray_new(2);
  
  carray_push(array, 1);
  carray_push(array, 2);
  carray_push(array, 3);

  
  assert(carray_find(array, 2) == 1);
  assert(carray_find(array, 4) == -1);
  
  carray_destroy(array);

}

void run_all_test() {
//  test_init_size();
//  test_push();
//  test_empty();
//  test_at();
//  test_insert();
//  test_prepend();
//  test_pop();
//  test_delete();
//  test_find();
//  test_resize();
  test_remove();
}




int main() {
  run_all_test();
}








