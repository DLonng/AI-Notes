#include "array_gp.h"

#include <string.h>
#include <stdbool.h>


Array* ArrayCreate()
{
    Array* array = malloc(sizeof(Array));
    if (NULL == array) {
        printf("ArrayCreate: array malloc memory fail!\n");
        return NULL;
    }
    
    array->p = NULL;

    array->size = 0;
    array->type_size = 0;
    array->len = 0;

    array->dup = NULL;
    array->free = NULL;
    array->match = NULL;

    printf("ArrayCreate\n");
    return array;
}

void ArrayInit(Array* array, int size, int type_size)
{
    if (NULL == array || size <= 0 || type_size <= 0)
        return ;
    
    array->p = calloc(1, size * type_size);
    if (NULL == array->p) {
        printf("ArrayInit: array->p calloc memory fail!\n");
        return ;
    }

    array->size = size;
    array->type_size = type_size;
    array->len = 0;

    printf("ArrayInit\n");
}

int ArrayInsert(Array* array, size_t pos, void* const value)
{
    if (NULL == array)
        return -1;
    
    if (array->len >= array->size)
        return -2;
    
    if (pos <= 0 || pos > array->size)
        return -3;
    
    // Move elements to next from pos index.
    char* p_begin = array->p;
    for (size_t i = array->len; i > pos - 1; i--) {
        void* p_new = p_begin + i * array->type_size;
        void* p_old = p_begin + (i - 1) * array->type_size;

        if (NULL != array->dup)
            array->dup(p_new, p_old);
        else
            memcpy(p_new, p_old, array->type_size);
    }

    // Insert *value to pos index.
    void* p_copy = (void*)(p_begin + (pos - 1) * array->type_size);
    
    if (NULL != array->dup)
        array->dup(p_copy, value);
    else
        memcpy(p_copy, value, array->type_size);

    array->len++;

    return 0;
}

size_t ArraySearchValue(Array* array, void* const value)
{
    if (NULL == array)
        return -1;
    
    char* p_begin = array->p;
    size_t i = 0;
    int cmp = 0;

    for (; i < array->len; i++) {
        if (NULL != array->match)
            cmp = array->match(p_begin + i * array->type_size, value);
        else
            cmp = memcmp(p_begin + i * array->type_size, value, array->type_size);

        // 0 is find!
        if (0 == cmp)
            break;
    }

    return i;
}

void* ArrayIndex(Array* array, size_t index)
{
    if (NULL == array)
        return NULL;
    
    if (index <= 0 || index > array->len)
        return NULL;
    
    char* p_begin = array->p;
    return p_begin + array->type_size * index;
}

int ArrayModify(Array* array, size_t pos, void* const value)
{
    if (NULL == array)
        return -1;
    
    if (pos <= 0 || pos > array->len)
        return -2;
    
    char* p_begin = array->p;
    void* p_old = p_begin + (pos - 1) * array->type_size;

    if (NULL != array->dup)
        array->dup(p_old, value);
    else
        memcpy(p_old, value, array->type_size);
    
    return 0;
}

void ArrayDelValue(Array* array, void* const value)
{
    if (NULL == array)
        return ;
    
    char* p_begin = array->p;
    bool copy = false;

    for (size_t i = 0; i < array->len; i++) {
        if (!copy) {
            int cmp = 0;
            if (NULL != array->match)
                cmp = array->match(p_begin + i * array->type_size, value);
            else
                cmp = memcmp(p_begin + i * array->type_size, value, array->type_size);
            
            if (0 == cmp) {
                copy = true;
                continue;
            }
        } else {
            void* p_old = p_begin + i * array->type_size;
            void* p_new = p_begin + (i - 1) * array->type_size;

            if (NULL != array->dup)
                array->dup(p_new, p_old);
            else
                memcpy(p_new, p_old, array->type_size);
        }
    }

    if (copy)
        array->len--;
}

void ArrayDelIndex(Array* array, size_t index)
{
    if (NULL == array)
        return ;
    
    if (index < 0 || index >= array->len)
        return ;
    
    char* p_begin = array->p;
    for (size_t i = index; i < array->len; i++) {
        void* p_old = p_begin + (i + 1) * array->type_size;
        void* p_new = p_begin + i * array->type_size;

        if (NULL != array->dup)
            array->dup(p_new, p_old);
        else
            memcpy(p_new, p_old, array->type_size);
    }

    array->len--;
}

size_t ArrayLen(Array* array)
{
    if (NULL == array)
        return 0;

    return array->len;
}

size_t ArraySize(Array* array)
{
    if (NULL == array)
        return 0;
    
    return array->size;
}


void ArrayEmpty(Array* array)
{
    if (NULL == array) {
        printf("ArrayEmpty\n");
        return ;
    }
    
    if (array->p != NULL) {
        free(array->p);
        array->p = NULL;
    }

    free(array);
    array = NULL;

    printf("ArrayEmpty\n");
}


void ArrayShow(Array* array) {
    for (size_t i = 0; i < array->len; i++)
        printf("%d ", ((int*)(array->p))[i]);

    printf("\n");
}

int main()
{
    Array* array = ArrayCreate();
    ArrayInit(array, 5, sizeof(int));

    int a = 1;
    ArrayInsert(array, 1, &a);
    a = 2;
    ArrayInsert(array, 2, &a);
    a = 3;
    ArrayInsert(array, 3, &a);
    a = 4;
    ArrayInsert(array, 4, &a);
    a = 5;
    ArrayInsert(array, 5, &a);

    ArrayShow(array);

    printf("%d index is %zu\n", a, ArraySearchValue(array, &a));

    size_t index = 2;
    printf("index %zu is %d\n", index, *((int*)(ArrayIndex(array, index))));

    int m = 30;
    // index == pos - 1
    if (0 == ArrayModify(array, 3, &m))
        printf("modify pos 3 is %d\n",  ((int*)(array->p))[2]);
    
    ArrayShow(array);

    printf("del 30\n");
    ArrayDelValue(array, &m);
    ArrayShow(array);

    printf("del index 1\n");
    ArrayDelIndex(array, 1);
    ArrayShow(array);

    ArrayEmpty(array);
    return 0;
}