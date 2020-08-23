#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/*
 * Get array min data address
 */
int* get_min_addr(int a[], int n) {
	int min = a[0];
	int *min_addr = NULL;

	for (int i = 0; i < n; i++) {
		if (a[i] < min) {
			min = a[i];
			min_addr = &a[i];
		}
	}

	return min_addr;
}

/*
 * Get array max data address
 */
int *get_max_addr(int a[], int n) {
	int max = a[0];
	int *max_addr = NULL;

	for (int i = 0; i < n; i++) {
		if (a[i] > max) {
			max = a[i];
			max_addr = &a[i];
		}
	}

	return max_addr;
}

void main() {
	time_t ts;
	srand((unsigned int)time(&ts));
	int a[10] = { 0 };
	
	for (int i = 0; i < 10;i++) {
		a[i] = rand() % 100;
		printf("%d %x\n", a[i], &a[i]);
	}

	printf("\n");
	printf("max: %d %x\n", *(get_min_addr(a, 10)), get_min_addr(a, 10));
	printf("min: %d %x\n", *(get_max_addr(a, 10)), get_max_addr(a, 10)); 
}