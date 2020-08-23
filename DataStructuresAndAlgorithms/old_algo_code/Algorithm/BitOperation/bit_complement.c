#include <stdio.h>
#include <stdlib.h>


void print_complement_for(int x) {
	int data = 1 << 31;
	for (int i = 1; i <= 32; i++) { 
		printf("%d", (x & data ? 1 : 0));
		x <<= 1;
		if (i % 4 == 0)
			printf(" ");
	} 
}

void print_complement_while(int x) {
	int data = 1 << 31;
	int i = 1;
	while (i <= 32) {
		printf("%d", (x & data ? 1 : 0));
		x <<= 1;
		if (i % 4 == 0)
			printf(" ");
		i++;
	}
}

void print_complement_do_while(int x) {
	int data = 1 << 31;
	int i = 1;
	do {
		printf("%d", (x & data ? 1 : 0));
		x <<= 1;
		if (i % 4 == 0)
			printf(" ");
		i++;
	} while (i <= 32);
}

void print_complement_goto(int x) {
	int data = 1 << 31;
	int i = 1;
A:
	{
		printf("%d", (x & data ? 1 : 0));
		x <<= 1;
		if (i % 4 == 0)
			printf(" ");
		i++;
		if (i <= 32)
			goto A;
	}
}



void print_complement_recursive(int num)
{
	// Can also use global var i.
	static int i = 1;
	int data = 1 << 31;
	if (i == 33) {
		return ; 
	} else {
		printf("%d", (num & data ? 1 : 0));
		num <<= 1;
		if (i % 4 == 0)
			printf(" ");
		i++;
		return print_complement_recursive(num);
	}
}

int main() {
	int num ;
	scanf("%d", &num);

	print_complement_for(num);
	putchar('\n');

	print_complement_while(num);
	putchar('\n');
	
	print_complement_do_while(num);
	putchar('\n');

	print_complement_goto(num);
	putchar('\n');

	print_complement_recursive(num);
	putchar('\n');

	return 0;
}