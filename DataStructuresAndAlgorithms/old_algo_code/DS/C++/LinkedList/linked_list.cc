#include "linked_list.h"

#include <iostream>

template<typename T>
LinkedList<T>::LinkedList() : head_(nullptr) {

}


template<typename T>
LinkedList<T>::~LinkedList() {
  Node<T> *current = head_;
  Node<T> *next = nullptr;

  while (current) {
    next = current->next();
	delete current;
	current = next;
  }
}



template<typename T>
const int LinkedList<T>::Size() {
  int size = 0;

  auto current = head_;

  while (current) {
    size++;
	current = current->next();
  }

  return size;

}



template<typename T>
bool LinkedList<T>::Empty() {
  return head_ == nullptr;
}


template<typename T>
const T& LinkedList<T>::ValueAt(int index) {
  auto current = head_;

  for (int i = 0; (i < index) && current; i++)
	  current = current->next();

  if (current == nullptr) {
    std::cout << "Index is out of bounds." << std::endl;
	exit(EXIT_FAILURE);
  }

  return current->data();
}



template<typename T>
void LinkedList<T>::PushFront(const T item) {
  auto new_node = new Node<T>(item);
  new_node->set_next(head_);
  head_ = new_node;
}



template<typename T>
void LinkedList<T>::PrintList() {
  std::cout << std::endl;
  
  for (auto current = head_; current; current = current->next()) {
    std::cout << current->data() << " -> ";
  }
  
  std::cout << std::endl;
}


template<typename T>
const T LinkedList<T>::PopFront() {
  T pop_value = head_->data();
  auto first = head_;
  head_ = head_->next();
  delete first;
  first = nullptr;
  return pop_value;
}


template<typename T>
void LinkedList<T>::PushBack(const T item) {
  auto new_node = new Node<T>(item);
  new_node->set_next(nullptr);
  
  auto current = head_;
  
  if (nullptr == current) {
    head_ = new_node;
  } else {
    while (current->next())
	  current = current->next();
  
    current->set_next(new_node);
  }
}



template<typename T>
const T LinkedList<T>::PopBack() {
  auto current = head_;
  Node<T> *prev = nullptr;

  while (current->next()) {
    prev = current;
	current = current->next();
  }

  T pop_value = current->data();
  prev->set_next(nullptr);
  
  delete current;
  current = nullptr;
  
  return pop_value;
}



template<typename T>
const T LinkedList<T>::Front() const {
  return head_->data();
}




template<typename T>
const T LinkedList<T>::Back() const {
  auto current = head_;
  while (current->next())
	  current = current->next();
  return current->data();
}


template<typename T>
void LinkedList<T>::Insert(int index, const T value) {
  auto current = head_;
  Node<T> *prev = nullptr;
  
  int i = 0;
  for (i = 0; (i < index) && current; i++) {
    prev = current;
	current = current->next();
  }

  if (i != index) {
    std::cout << "Index out of bounds" << std::endl;
	exit(EXIT_FAILURE);
  }

  auto new_node = new Node<T>(value);

  if (prev != nullptr) {
    new_node->set_next(prev->next());
	prev->set_next(new_node);
  } else {
    new_node->set_next(head_);
	head_ = new_node;
  }
}




template<typename T>
void LinkedList<T>::Erase(const int index) {
  auto current = head_;
  Node<T> *prev = nullptr;

  int i = 0;
  for (i = 0; (i < index) && current; i++) {
    prev = current;
	current = current->next();
  }

  if (i != index) {
    std::cout << "Index out of bounds" << std::endl;
	exit(EXIT_FAILURE);
  }
  
  if (prev)
    prev->set_next(current->next());
  else
    head_ = current->next();
  
  delete current;
  current = nullptr;
}



template<typename T>
T LinkedList<T>::ValueNFromEnd(const int n) const {
  if ((n < 1)) {
    std::cout << "ValueNFromEnd(n): n > 0" << std::endl;
	exit(EXIT_FAILURE);
  }
	
  auto current = head_;
  auto match = head_;
 
  int i = 0;
  for (i = 0; (i < n) && current; i++)
    current = current->next();

  if (i != n) {
    std::cout << "Index out of bound" << std::endl;
	exit(EXIT_FAILURE);
  }
  
  while (current) {
    current = current->next();
	match = match->next();
  }

  return match->data();
}



template<typename T>
void LinkedList<T>::Reverse() {
  auto current = head_;
  Node<T> *prev = nullptr;
  Node<T> *next = nullptr;

  while (current) {
    next = current->next();
	current->set_next(prev);
	prev = current;
	current = next;
  }
  
  head_ = prev;
}

template<typename T>
void LinkedList<T>::RemoveValue(const T value) {
  auto current = head_;
  Node<T> *prev = nullptr;

  while (current) {
    if (value == current->data()) {
	  if (prev)
	    prev->set_next(current->next());
	  else 
	    head_ = current->next();
	  delete current;
	  current = nullptr;
	  break;
	} else {
	  prev = current;
	  current = current->next();
	}
  }
}


