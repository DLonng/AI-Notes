#include <stdio.h>

/*
 * Determine whether num is a power of 2
 * eg:
 *	num = 4
 *		0010 & (0010 - 1 = 0001) = 0000 => return 1;
 *	num = 12
 *		0110 & (0110 - 1 = 0101) = 0100 => return 0;
 */
static int is_two_power(int num) {
	return (num) & (num - 1) ? 0 : 1; 
}


void main() {
	int a = 0;
	scanf("%d", &a);
	printf("%d\n", is_two_power(a));
}