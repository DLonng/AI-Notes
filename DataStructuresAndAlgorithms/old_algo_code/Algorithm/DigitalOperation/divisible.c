#include <stdio.h>


static void show_divisible(int low, int high, int div) {
	for (int i = low; i <= high; i++) {
		if (i % div == 0)
			continue;
		printf("%d ", i);
	}
	putchar('\n');
}


void main() {
	show_divisible(100, 200, 3);
}