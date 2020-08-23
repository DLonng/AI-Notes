#ifndef LINK_LIST_H_
#define LINK_LIST_H_

#include "node.h"


template<typename T>
class LinkedList {
 public:
  LinkedList();
  ~LinkedList();
  const int Size();
  bool Empty();
  const T& ValueAt(int index);
  void PushFront(const T item);
  void PrintList();
  const T PopFront();
  void PushBack(const T item);
  const T PopBack();
  const T Front() const;
  const T Back() const;
  void Insert(int index, const T value);
  void Erase(const int index);
  T ValueNFromEnd(const int n) const;
  void Reverse();
  void RemoveValue(const T value);
 private:
  Node<T> *head_;

};


#endif //LINK_LIST_H_
