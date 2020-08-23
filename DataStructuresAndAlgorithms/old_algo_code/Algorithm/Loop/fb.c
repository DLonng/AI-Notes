#include <stdio.h>
#include <stdlib.h>
/*
 * Count Fibonacci
 */
void fb_while(int n) {
	int f1 = 1;
	int f2 = 1;

	printf("%d ", f1);
	printf("%d ", f2);	

	int f3 = 0;
	int i = 2;
	while (i < n) {
		f3 = f1 + f2;
		printf("%d ", f3);
		f1 = f2;
		f2 = f3;
		i++;
	} 
}

void fb_do_while(int n)
{
	int f1 = 1;
	int f2 = 1;

	printf("%d ", f1);
	printf("%d ", f2);

	int f3 = 0;
	int i = 2;
	do {
		f3 = f1 + f2;
		printf("%d ", f3);
		f1 = f2;
		f2 = f3;
		i++;
	} while (i < n);
}

void fb_for(int n) {
	int f1 = 1;
	int f2 = 1;

	printf("%d ", f1);
	printf("%d ", f2);

	int f3 = 0;
	for (int i = 2; i < n; i++) {
		f3 = f1 + f2;
		printf("%d ", f3);
		f1 = f2;
		f2 = f3; 
	} 
}

void fb_rec(int n) {
	static int i = 2;

	static int f1 = 1;
	static int f2 = 1;
	if (2 == i) {
		printf("%d ", f1);
		printf("%d ", f2);
	}

	int f3 = 0;
	if (i >= n) {
		return;
	} else { 
		f3 = f1 + f2;
		printf("%d ", f3);
		f1 = f2;
		f2 = f3;
		i++;
		fb_rec(n); 
	}
}

int main(void) { 
	fb_while(10);
	putchar('\n');

	fb_do_while(10);
	putchar('\n');

	fb_for(10);
	putchar('\n');
	 
	fb_rec(10);
	putchar('\n');
	return 0;
}