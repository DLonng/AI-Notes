#ifndef __ARRAY_GP_H__
#define __ARRAY_GP_H__

#include <stdio.h>
#include <stdlib.h>

typedef struct Array
{
    size_t size;
    size_t len;
    size_t type_size;

    void (*dup)(void* ptr, void* key);
    void (*free)(void* ptr);
    int (*match)(void* ptr, void* key);

    void* p;
} Array;


#define ArraySetDupMethod(a, m) ((a)->dup = (m))
#define ArraySetFreeMethod(a, m) ((a)->free = (m))
#define ArraySetMatchMethod(a, m) ((a)->match = (m))

#define ArrayGetDupMethod(a) ((a)->dup)
#define ArrayGetFree(a) ((a)->free)
#define ArrayGetMatchMethod(a) ((a)->match)

Array* ArrayCreate();
void ArrayInit(Array* array, int size, int type_size);

int ArrayInsert(Array* array, size_t pos, void* const value);
size_t ArraySearchValue(Array* array, void* const value);
void* ArrayIndex(Array* array, size_t index);
int ArrayModify(Array* array, size_t pos, void* const value);

size_t ArrayLen(Array* array);
size_t ArraySize(Array* array);

void ArrayEmpty(Array* array);
void ArrayDelValue(Array* array, void* const value);
void ArrayDelIndex(Array* array, size_t index);
void ArrayShow(Array* array);

#endif // __ARRAY_GP_H__