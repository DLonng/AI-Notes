#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// static array
struct array {
    int* arr;
    int size;
    int used;
};

// Alloc memory for int array.
void alloc(struct array* array)
{
    array->arr = (int*)malloc(array->size * sizeof(int));
}

// Insert a elem to array.
int insert(struct array* array, int elem)
{
    if (array->used >= array->size)
        return -1;

    int index = 0;
    
    for (index = 0; index < array->used; index++) {
        if (array->arr[index] > elem)
            break;
    }

    // void *memmove(void *dest, const void *src, size_t n)
    if (index < array->used)
        memmove(&array->arr[index + 1], &array->arr[index], (array->used - index) * sizeof(int));

    array->arr[index] = elem;
    array->used++;
    
    return index;
}

// Delete a elem in index.
int delete(struct array* array, int index)
{
    if (index < 0 || index >= array->used)
        return -1;
    
    // array->used is length (1, 2, 3, ...), but index is (0, 1, 2, ...)
    // so need to sub 1 
    memmove(&array->arr[index], &array->arr[index + 1], (array->used - index - 1) * sizeof(int));

    array->used--;

    return 0;
}

// Search a elem.
int search(struct array* array, int elem)
{
    for (int i = 0; i < array->used; i++) {
        if (array->arr[i] == elem)
            return i;
        
        if (array->arr[i] > elem)
            return -1;
    }

    return -1;
}

// Show array
void show_array(struct array* array)
{
    for (int i = 0; i < array->used; i++)
        printf("array[%d] = %d\n", i, array->arr[i]);
}

// gcc array.c
// ./a.out
int main() 
{
    struct array ten_int = {NULL, 10, 0};
    
    alloc(&ten_int);
    if (ten_int.arr == NULL) {
        printf("Array alloc fail.\n");
        return -1;
    }

    // 1
    insert(&ten_int, 1);
    // 1 3
    insert(&ten_int, 3);
    // 1 2 3
    insert(&ten_int, 2);

    printf("insert 1, 3, 2\n");
    // 1 2 3
    show_array(&ten_int);

    int index = search(&ten_int, 2);
    printf("\n2 is at position %d\n", index);

    index = search(&ten_int, 9);
    printf("9 is at position %d\n", index);

    printf("\ndelete index = 6 element \n");
    delete(&ten_int, 6);
    show_array(&ten_int);

    printf("\ndelete index = 0 element \n");
    delete(&ten_int, 0);
    show_array(&ten_int);

    return 0;
}