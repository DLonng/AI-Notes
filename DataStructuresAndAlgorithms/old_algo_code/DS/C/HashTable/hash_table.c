#include "hash_table.h"

#include <stdlib.h>
#include <string.h>

int m = 0;

HashTable *hash_new(int table_size) {
  HashTable *table = (HashTable *)malloc(sizeof(HashTable));
  table->count = table_size;
  m = table_size;
  table->data = (int *)malloc(sizeof(int) * table_size);
  memset(table->data, 0, table->count);
  return table;
}

int hash(int key) {
  return key % m;
}

void hash_add(HashTable *table, int key) {
  int addr = hash(key);
  while (table->data[addr] != 0)
    addr = hash(addr + 1);
  table->data[addr] = key;
}

int hash_exists(HashTable *table, int key) {
  int addr = hash(key);
  
  while (table->data[addr] != key) {
    addr = hash(addr + 1);
	if (addr == hash(key))
	  return 0;
  }

  return 1;
}

int hash_get(HashTable *table, int key) {
  if (!hash_exists(table, key)) {
    printf("key is no exists in hash table.\n");
	exit(EXIT_FAILURE);
  }

  int addr = hash(key);

  while (table->data[addr] != key)
    addr = hash(addr + 1);
  
  return addr;
}

void hash_remove(HashTable *table, int key) {
  if (!hash_exists(table, key)) {
    printf("key is no exists in hash table.\n");
	exit(EXIT_FAILURE);
  }
 
  int addr = hash(key);

  while (table->data[addr] != key)
    addr = hash(addr + 1);
  
  table->data[addr] = 0;
}


void print_table(HashTable *table) {
  printf("count: %d\n", table->count);
  
  for (int i = 0; i < table->count; i++) {
    printf("%d ", table->data[i]);
  }

  putchar('\n');
}



