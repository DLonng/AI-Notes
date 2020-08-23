#include "linked_queue.h"




template <typename T> 
LinkQueue<T>::LinkQueue() : head_(nullptr), tail_(nullptr) { }

template <typename T> 
LinkQueue<T>::~LinkQueue() { }

template <typename T> 
void LinkQueue<T>::Enqueue(const T value) {
  auto new_node = new Node<T>(value);
  
  if (tail_ != nullptr)
    tail_->set_next(new_node);
  
  tail_ = new_node;
  
  if (nullptr == head_)
    head_ = new_node;
}

template <typename T> 
const T LinkQueue<T>::Dequeue() {
  if (nullptr == head_) {
    std::cout << "Dequeue from empty queue." << std::endl;
	exit(1);
  }

  T value = head_->data();
  auto tmp_head = head_;
  head_ = head_->next();
  
  delete tmp_head;
  tmp_head = nullptr;

  return value;
}

template <typename T> 
bool LinkQueue<T>::Empty() {
  return head_ == nullptr;
}

template <typename T> 
void LinkQueue<T>::PrintQueue() {
  auto cur = head_;
  while (cur) {
    std::cout << cur->data() << " -> ";
	cur = cur->next();
  }
  std::cout << std::endl;
}






