#include <stdio.h>




/*
 * 2 / 1 + 3 / 2 + 5 / 3 + ... 
 */
static double array_sum(int n) {
	float a = 1;
	float b = 2;
	float t = 0;
	double sum = 0;
	
	for (int i = 0; i < n; i++) {
		sum = sum + b / a;
		t = b;
		b = a + b;
		a = t;
	}

	return sum;
}


void main() {
	printf("%f\n", array_sum(3));
}