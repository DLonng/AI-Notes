#ifndef HASH_TABLE_H_
#define HASH_TABLE_H_

#include <stdio.h>


typedef struct HashTable {
  int *data;
  int count;
} HashTable;


HashTable *hash_new(int table_size);
int hash(int key);
void hash_add(HashTable *table, int key);
int hash_exists(HashTable *table, int key);
int hash_get(HashTable *table, int key);
void hash_remove(HashTable *table, int key);
void print_table(HashTable *table);
#endif // HASH_TABLE_H_



