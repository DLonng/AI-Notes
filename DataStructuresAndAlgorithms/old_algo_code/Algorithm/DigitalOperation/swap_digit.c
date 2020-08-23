#include <stdio.h>
#include <stdlib.h>

/*
	*a = 1, *b = 2;
	=>
	tmp = 1;
	*a = 2;
	*b = 1;
	=>
	*a = 2, *b = 1;
*/
static void normal_swap(int *a, int *b) {
	int tmp = *a;
	*a = *b;
	*b = tmp;
}

/*
	*a = 1, *b = 2;
	=>
	(*a) = (*a) ^ (*b) => *a = 0011
	(*b) = (*a) ^ (*b) => *b = 0001 = 1
	(*a) = (*a) ^ (*b) => *a = 0010 = 2
	=>
	*a = 2, *b = 1;
*/
static void xor_swap(int *a, int *b) {
	(*a) = (*a) ^ (*b);
	(*b) = (*a) ^ (*b);
	(*a) = (*a) ^ (*b);
}

/*
	*a = 1, *b = 2;
	=>
	(*a) = (*a) + (*b) => *a = 3
	(*b) = (*a) - (*b) => *b = 3 - 2 = 1
	(*a) = (*a) - (*b) => *a = 3 - 1 = 2
	=>
	*a = 2, *b = 1;
*/
static void add_swap(int *a, int *b) {
	(*a) = (*a) + (*b);
	(*b) = (*a) - (*b);
	(*a) = (*a) - (*b);
}

void main() {
	int a = 1;
	int b = 2;
	printf("a = %d, b = %d\n", a, b);

	normal_swap(&a, &b);
	printf("a = %d, b = %d\n", a, b);
	
	xor_swap(&a, &b);
	printf("a = %d, b = %d\n", a, b);
	
	add_swap(&a, &b);
	printf("a = %d, b = %d\n", a, b);
}