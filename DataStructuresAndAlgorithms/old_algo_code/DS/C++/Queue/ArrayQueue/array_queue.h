#ifndef ARRAY_QUEUE_H_
#define ARRAY_QUEUE_H_

#include <iostream>

const int kMaxSize = 5;
const int kMaxPos = kMaxSize + 1;

class ArrayQueue {
 public:
  ArrayQueue();
  ~ArrayQueue();
  void Enqueue(const int item);
  const int Dequeue();
  bool Empty();
  bool Full();
 private:
  int head_;
  int tail_;
  int data_[kMaxPos];
};




#endif //ARRAY_QUEUE_H_
