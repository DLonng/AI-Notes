#include "c_array.h"


const int kMinInitCapacity = 1;
const int kMinCapacity = 16;
const int kGrowthFactor = 2;
const int kShrinkFactor = 4;


CArray *carray_new(int capacity) {
  // Adjust capacity to the power of 2 - 16, 32, 64, 128
  int really_capacity = carray_modify_capacity(capacity);
  
  CArray *array = (CArray *)malloc(sizeof(CArray));
  check_address(array);

  array->data = (int *)malloc(sizeof(int) * really_capacity);
  check_address(array->data);

  array->capacity = really_capacity;
  array->size = 0;
}


void carray_destroy(CArray *arrptr) {
  free(arrptr->data);
  arrptr->data = NULL;
  free(arrptr);
  arrptr = NULL;
}



int carray_modify_capacity(int capacity) {
  if (capacity < kMinInitCapacity) {
    exit(-1);
  }
  else if (capacity < kMinCapacity) {
    return kMinCapacity;
  }
  else {
    int really_capacity = kMinInitCapacity;
    // capacity <= really_capacity / 2，break while loop.
    //while (capacity > (really_capacity / kGrowthFactor))
    while (capacity > really_capacity)
      really_capacity *= kGrowthFactor;

    return really_capacity;
  }
}


void check_address(void *address) {
  if (NULL == address) {
    printf("Allocate memory failed.\n");
	exit(-1);
  }
}


void print_carray(CArray *arrptr) {
  printf("\nsize = %d\n", arrptr->size);
  printf("capacity = %d\n\n", arrptr->capacity);
  for (int i = 0; i < arrptr->size; i++)
    printf("data[%d] = %d\n", i, arrptr->data[i]);

  putchar('\n');
}



int carray_size(CArray *arrptr) {
  assert((arrptr != NULL) && (arrptr->data != NULL));
  return arrptr->size;
}


int carray_capacity(CArray *arrptr) {
  assert((arrptr != NULL) && (arrptr->data != NULL));
  return arrptr->capacity;
}


int carray_is_empty(CArray *arrptr) {
  return !carray_size(arrptr);
}


int carray_at(CArray *arrptr, int index) {
  assert((arrptr != NULL) && (arrptr->data != NULL));
  return arrptr->data[index];
}


void carray_push(CArray *arrptr, int item) {
  assert((arrptr != NULL) && (arrptr->data != NULL));
  
  carray_resize(arrptr, arrptr->size + 1);

  arrptr->data[carray_size(arrptr)] = item; 
  arrptr->size++;
}

void carray_insert(CArray *arrptr, int index, int item) {
  assert((arrptr != NULL) && (arrptr->data != NULL));
  assert((index >= 0) && (index <= arrptr->size - 1));
  
  carray_resize(arrptr, arrptr->size + 1);

  int last_index = carray_size(arrptr) - 1;
  
  for (int i = last_index; i >= index; i--) {
    arrptr->data[i + 1] = arrptr->data[i];
  }
  // Insert...
  arrptr->data[index] = item;
  arrptr->size++;
}

void carray_prepend(CArray *arrptr, int item) {
  carray_insert(arrptr, 0, item);
}


int carray_pop(CArray *arrptr) {
  assert((arrptr != NULL) && (arrptr->data != NULL) && (arrptr->size != 0));

  carray_resize(arrptr, arrptr->size - 1);

  int last = arrptr->data[carray_size(arrptr) - 1];
  arrptr->size--;
  return last;
}


void carray_delete(CArray *arrptr, int index) {
  assert((arrptr != NULL) && (arrptr->data != NULL));
  assert((index >= 0) && (index <= arrptr->size - 1));
  
  carray_resize(arrptr, arrptr->size - 1);
  
  int last_index = carray_size(arrptr) - 1;
  
  for (int i = index; i < last_index; i++) {
	  arrptr->data[i] = arrptr->data[i + 1];
  }
  
  arrptr->size--;
}


void carray_remove(CArray *arrptr, int item) {
  int end_index = carray_size(arrptr) - 1;
  for (int i = 0; i < end_index; i++) {
    if (item == arrptr->data[i]) 
		carray_delete(arrptr, i--);
  }
}

int carray_find(CArray *arrptr, int item) {
  assert((arrptr != NULL) && (arrptr->data != NULL));
  int len = carray_size(arrptr);

  for (int i = 0; i < len; i++) {
    if (item == arrptr->data[i])
		return i;
  }

  return -1;
}


void carray_resize(CArray *arrptr, int candidate_size) {
  assert((arrptr != NULL) && (arrptr->data != NULL));
  if (arrptr->size < candidate_size) {
    if (arrptr->size == arrptr->capacity) {
	  carray_up_size(arrptr);
	}
  } else if (arrptr->size > candidate_size) {
    if (arrptr->size < arrptr->capacity / kShrinkFactor) {
	  carray_down_size(arrptr);
	}
  }
}


void carray_up_size(CArray *arrptr) {
  int old_capacity = arrptr->capacity;
  int new_capacity = old_capacity * 2;
  int *new_data = (int *)malloc(sizeof(int) * new_capacity);
  check_address(new_data);

  for (int i = 0; i < arrptr->size; i++) {
    new_data[i] = arrptr->data[i];
  }

  free(arrptr->data);
  arrptr->data = new_data;
  arrptr->capacity = new_capacity;
}



void carray_down_size(CArray *arrptr) {
  int old_capacity = arrptr->capacity;
  int new_capacity = arrptr->capacity / 2;
  
  if (new_capacity < kMinCapacity) {
    new_capacity = kMinCapacity;
  }
  
  if (new_capacity != old_capacity) {
    int *new_data = (int *)malloc(sizeof(int) * new_capacity);
	
	check_address(new_data);
	
	for (int i = 0; i < arrptr->size; i++) {
	  new_data[i] = arrptr->data[i];
	}

	free(arrptr->data);
	arrptr->data = new_data;
	arrptr->capacity = new_capacity;
  }

}


