#include <stdio.h>
#include <stdlib.h>


static void bit_low_case(char string[], int length) {
	// The fifth digit of the lowercase letter is 1, 0010 0000 = 0x20
	for (int i = 0; i < length; i++) 
		string[i] |= 0x20;
}

static void bit_high_case(char string[], int length) {
	// The fifth digit of the highcase letter is 0, 1101 1111 = 0xDF
	for (int i = 0; i < length; i++) 
		string[i] &= 0xDF;
}


int main(void) {
	char a[] = "BaSiC";
	char b[] = "MinIx";

	printf("a = %s, b = %s\n", a, b);
	
	bit_low_case(a, sizeof(a));
	bit_high_case(b, sizeof(b));
	
	printf("a = %s, b = %s\n", a, b);
	return 0;
}
