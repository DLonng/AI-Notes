#include <stdio.h>

/*
	a + b 
	<=> 
	a ^ b , (a & b) << 1

eg:
	8 + 6 = 14
	a ^ b = 1000 ^ 0110 = 1110
	(a & b) << 1 = (1000 & 0110) << 1 = 0, no carry.
*/
static int bit_add(int a, int b) {
	if (0 == a)
		return b;
	
	if (0 == b)
		return a;
	
	// Using ^ to add
	int res = a ^ b;
	// Get number of carry.
	int wei = (a & b) << 1;
	return bit_add(res, wei);
		 
}


void main() {
	printf("10 + 6 = %d\n", bit_add(10, 6));
}