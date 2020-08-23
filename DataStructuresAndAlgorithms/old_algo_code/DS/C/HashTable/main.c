#include "hash_table.h"

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

void test_new() {
  HashTable *table = hash_new(5);
  print_table(table);
  free(table);
}

void test_hash_add() {
  HashTable *table = hash_new(5);
  
  hash_add(table, 12);
  hash_add(table, 67);
  hash_add(table, 56);
  hash_add(table, 16);
  hash_add(table, 25);

  print_table(table);
  free(table);
}

void test_hash_exists() {
  HashTable *table = hash_new(5);
  
  hash_add(table, 12);
  hash_add(table, 67);
  hash_add(table, 56);
  hash_add(table, 16);
  hash_add(table, 25);

  assert(hash_exists(table, 12));
  assert(hash_exists(table, 67));
  assert(hash_exists(table, 56));
  assert(hash_exists(table, 16));
  assert(hash_exists(table, 25));

  free(table);
}

void test_hash_get() {
  HashTable *table = hash_new(5);
  
  hash_add(table, 12);
  hash_add(table, 67);
  hash_add(table, 56);
  hash_add(table, 16);
  hash_add(table, 25);

  print_table(table);
  assert(hash_get(table, 12) == 2);
  assert(hash_get(table, 67) == 3);
  assert(hash_get(table, 56) == 1);
  assert(hash_get(table, 16) == 4);
  assert(hash_get(table, 25) == 0);
  free(table);
}

void test_remove() {
  HashTable *table = hash_new(5);
  
  hash_add(table, 12);
  hash_add(table, 67);
  hash_add(table, 56);
  hash_add(table, 16);
  hash_add(table, 25);

  print_table(table);
  
  hash_remove(table, 12);
  hash_remove(table, 67);
  hash_remove(table, 56);
  hash_remove(table, 16);
  hash_remove(table, 25);
  print_table(table);
  
  free(table);
}




void run_all_test() {
  //test_new();
  //test_hash_add();
  //test_hash_exists();
  //test_hash_get();
  test_remove();
}

/**
 * @synopsis  main 
 *
 * @return   
 */
int main(void)
{
  run_all_test();	
  return 0;
}
