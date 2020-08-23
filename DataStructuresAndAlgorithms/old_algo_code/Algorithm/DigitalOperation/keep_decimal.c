#include <stdio.h>

void main() {
	float a = 12.34567;
	// b = 12.345
	float b = ((int)(a * 1000)) / 1000.0;
	printf("%f\n", b);
}