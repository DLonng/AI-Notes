#include <stdio.h>

/*
 * Detemine whether the num is narcissus.
 * Note:
 * 		num: [100, 999]
 * All: 153，370，371，407
 */
static int narcissus(int num) {
	static int a = 0;
	static int b = 0;
	static int c = 0;

	if (num == a^3 + b^3 + c^3) {
		return 1;
	} else {
		a = num % 10;
		b = num / 10 % 10;
		c = num / 100;
		return narcissus(num);
	}

	return 0;
}

int main() {

	printf("%d\n", narcissus(153));
	printf("%d\n", narcissus(370));
	printf("%d\n", narcissus(371));
	printf("%d\n", narcissus(407));
	
	return 0;
}