#ifndef MY_ARRAY_H_
#define MY_ARRAY_H_


#include <assert.h>
#include <stdio.h>
#include <stdlib.h>


typedef struct CArray {
  int *data;
  int size;
  int capacity;
} CArray;


// Create a new array with capacity.
CArray *carray_new(int capacity);

// Delete arrptr memory.
void carray_destroy(CArray *arrptr);

// Modify user input capacity to power of 2 - 16, 32, 64, 128
int carray_modify_capacity(int capacity);

// Check address wheather is NULL or no.
void check_address(void *address);

// Output arrptr something.
void print_carray(CArray *arrptr);

// Return myarray size.
int carray_size(CArray *arrptr);

// Return myarray capacity.
int carray_capacity(CArray *arrptr);

// Return true if myarray is empty.
int carray_is_empty(CArray *arrptr);

// Return arrptr->data[index] value.
int carray_at(CArray *arrptr, int index);

// Input item to arrptr->data
void carray_push(CArray *arrptr, int item);

// Insert item to arrptr->data[index].
void carray_insert(CArray *arrptr, int index, int item);

// Insert item to arrptr header.
void carray_prepend(CArray *arrptr, int item);

// Return the last value.
int carray_pop(CArray *arrptr);

// Delete arrptr->data[index].
void carray_delete(CArray *arrptr, int index);

// Remove all item in arrptr.
void carray_remove(CArray *arrptr, int item);

// If find then return 1, otherwise return 0.
int carray_find(CArray *arrptr, int item);

// Call up_size and down_size to resize for logic.
void carray_resize(CArray *arrptr, int candidate_size);

// Resize to power of 2 - 16, 32, 64...
void carray_up_size(CArray *arrptr);

// Down size to half if size if 1 / 4 of the capacity.
void carray_down_size(CArray *arrptr);


#endif //MY_ARRAY_H_



