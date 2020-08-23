#include <stdio.h>

static int num_of_digits_do_while(int num) {
	int n = 0;
	do {
		num /= 10;
		n++;
	} while (num);
	
	return n;
}

static int num_of_digits_while(int num) {
	int n = 0;
	while (num) {
		num /= 10;
		n++;
	}
	
	return n;
}

static int num_of_digits_for(int num) {
	int n=0;
	for (; num != 0; num /= 10)
		n++;
	
	return n;
}

static int num_of_digits_rec(int num) {
	if (num < 10)
		return 1;
	else
		return 1 + num_of_digits_rec(num / 10);
}

int main() {
	printf("%d\n", num_of_digits_do_while(123));
	printf("%d\n", num_of_digits_while(123));
	printf("%d\n", num_of_digits_for(123));
	printf("%d\n", num_of_digits_rec(123));
	return 0;
}