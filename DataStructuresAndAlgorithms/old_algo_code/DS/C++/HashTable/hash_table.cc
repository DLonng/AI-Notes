#include "hash_table.h"

#include <cstring>
#include <cassert>

#include <iostream>

HashTable::HashTable(const int table_size) : count_(table_size), m_(table_size){
  assert(table_size > 0);
  data_ = new int[table_size]; 
  memset(data_, 0, sizeof(int) * count_);
}

HashTable::~HashTable() {
  delete data_;
}



const int HashTable::Hash(const int key) {
  return key % m_;
}



void HashTable::Add(const int key) {
  int addr = Hash(key);
  
  while (data_[addr] != 0)
    addr = Hash(addr + 1);
  
  data_[addr] = key;
}



const int HashTable::Get(const int key) {
  if (!Exist(key)) {
    std::cout << "Key don`t exist in this table." << std::endl;
	exit(EXIT_FAILURE);
  }
  
  int addr = Hash(key);
  while (data_[addr] != key)
    addr = Hash(addr + 1);

  return addr;

}


const int HashTable::Exist(const int key) {
  int addr = Hash(key);

  while (data_[addr] != key) {
    addr = Hash(addr + 1);
	if (addr == Hash(key))
      return 0;
  }

  return 1;
}


void HashTable::Remove(const int key) {
  if (!Exist(key)) {
    std::cout << "Key don`t exist in this table." << std::endl;
	exit(EXIT_FAILURE);
  }
  
  int addr = Hash(key);
  while (data_[addr] != key)
    addr = Hash(addr + 1);

  data_[addr] = 0;
}

void HashTable::PrintDebug() {
  std::cout << "count_ : " << count_ << std::endl;
  for (int i = 0; i < count_; i++)
    std::cout << data_[i] << " ";
  
  std::cout << std::endl;
}

