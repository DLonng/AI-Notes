#include "array_queue.h"


ArrayQueue::ArrayQueue() : head_(0), tail_(0) {

}

ArrayQueue::~ArrayQueue() {}

void ArrayQueue::Enqueue(const int item) {
  if (Full()) {
    std::cout << "This queue is full." << std::endl;
	exit(EXIT_FAILURE);
  }

  data_[tail_] = item;
  tail_ = (tail_ + 1) % kMaxPos;
}

const int ArrayQueue::Dequeue() {
  if (Empty()) {
    std::cout << "This queue is empty." << std::endl;
	exit(EXIT_FAILURE);
  }

  int value = data_[head_];
  data_[head_] = 0;
  head_ = (head_ + 1) % kMaxPos;
  return value;
}

bool ArrayQueue::Empty() {
  return head_ == tail_;
}

bool ArrayQueue::Full() {
  return (tail_ + 1) % kMaxPos == head_;
}
