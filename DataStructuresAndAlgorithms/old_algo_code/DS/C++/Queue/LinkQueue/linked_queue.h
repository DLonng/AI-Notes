#ifndef LINK_QUEUE_H_
#define LINK_QUEUE_H_

#include <iostream>
#include "node.h"


template <typename T> 
class LinkQueue {
 public:
  LinkQueue();
  ~LinkQueue();
  void Enqueue(const T value);
  const T Dequeue();
  bool Empty();
  void PrintQueue();

 private:
  Node<T> *head_;
  Node<T> *tail_;

};






#endif // LINK_QUEUE_H_

