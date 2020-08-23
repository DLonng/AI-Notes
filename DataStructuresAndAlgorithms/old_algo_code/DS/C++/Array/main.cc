
#include "cvector.h"
#include <iostream>
#include <cassert>


void test_print() {
  cspace::CVector vec(10);
  vec.Push(1);
  vec.Push(2);
  vec.Push(3);
  vec.PrintVector();
 
}


void test_size() {
  cspace::CVector vec(10);
  assert(vec.size() == 0);
  vec.Push(1);
  vec.Push(2);
  vec.Push(3);
  assert(vec.size() == 3);
}

void test_capacity() {
  cspace::CVector vec1(0);
  assert(vec1.capacity() == 1);

  cspace::CVector vec2(15);
  assert(vec2.capacity() == 16);
 
  cspace::CVector vec3(17);
  assert(vec3.capacity() == 32);
}

void test_is_empty() {
  cspace::CVector vec(10);
  assert(vec.IsEmpty() == 1);
  vec.Push(1);
  vec.Push(2);
  vec.Push(3);
  assert(vec.IsEmpty() == 0);
}

void test_at() {
  cspace::CVector vec(10);
  vec.Push(1);
  vec.Push(2);
  vec.Push(3);
  assert(vec.At(0) == 1);
  assert(vec.At(1) == 2);
  assert(vec.At(2) == 3);
}


void test_push() {
  cspace::CVector vec(10);
  assert(vec.capacity() == 16);
  for (int i = 0; i < 20; i++)
    vec.Push(i + 1);

  assert(vec.capacity() == 32);
  assert(vec.size() == 20);
}

void test_insert() {
  cspace::CVector vec(10);
  for (int i = 0; i < vec.capacity(); i++)
    vec.Push(i + 1);
  
  vec.Insert(1, 22);
  assert(vec.capacity() == 32);
  assert(vec.At(1) == 22);
}

void test_prepend() {
  cspace::CVector vec(10);
  assert(vec.size() == 0);
  vec.Push(1);
  vec.Push(2);
  vec.Push(3);
  vec.Prepend(22);
  assert(vec.size() == 4);
  assert(vec.At(0) == 22);
  assert(vec.At(1) == 1);
}

void test_pop() {
  cspace::CVector vec(10);
  vec.Push(1);
  vec.Push(2);
  vec.Push(3);
  assert(vec.size() == 3);
  vec.Pop();
  vec.Pop();
  vec.Pop();
  assert(vec.size() == 0);
}

void test_delete() {
  cspace::CVector vec(10);
  vec.Push(1);
  vec.Push(2);
  vec.Push(3);
  assert(vec.size() == 3);
  vec.Delete(1);
  assert(vec.At(1) == 3);
  assert(vec.size() == 2);
}



void test_remove() {
  cspace::CVector vec(10);
  vec.Push(1);
  vec.Push(2);
  vec.Push(2);
  vec.Push(2);
  vec.Push(2);
  vec.Push(3);

  vec.Remove(2);

  assert(vec.size() == 2);
  assert(vec.Find(2) == -1);
}

void test_find() {
  cspace::CVector vec(10);
  vec.Push(1);
  vec.Push(2);
  vec.Push(3);

  assert(vec.Find(1) == 0);
  assert(vec.Find(4) == -1);
}
/*
void test_resize_up() {
  cspace::CVector vec(10);
  for (int i = 0; i < vec.capacity(); i++)
	  vec.Push(i + 1);

  assert(vec.size() == 16);

  vec.Resize(18);
  
  assert(vec.size() == 32);
  assert(vec.At(15) == 16);
}


void test_resize_down() {
  cspace::CVector vec(30);
  for (int i = 0; i < 5; i++)
	  vec.Push(i + 1);
  assert(vec.size() == 5);
  assert(vec.capacity() == 32);

  vec.Resize(15);
  
  assert(vec.capacity() == 16);
}
*/

void run_all_test() {
  //test_print();
  //test_size();
  //test_capacity();
  //test_is_empty();
  //test_at();
  //test_push();
  //test_insert();
  //test_prepend();
  //test_pop();
  //test_delete();
  test_remove();
  //test_find();
  //test_resize_up();
  //test_resize_down();
}

int main(void) {
  run_all_test();
  return 0;
}
