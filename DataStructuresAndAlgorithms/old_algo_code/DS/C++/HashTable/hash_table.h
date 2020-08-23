#ifndef HASH_TABLE_H
#define HASH_TABLE_H



class HashTable {
 public:
  HashTable(const int table_size);
  ~HashTable();
  const int Hash(const int key);
  void Add(const int key);
  const int Get(const int key);
  const int Exist(const int key);
  void Remove(const int key);
  void PrintDebug();
 private:
  int count_;
  int m_;
  int *data_;
};








#endif //HASH_TABLE_H

