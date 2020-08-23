#include <stdio.h>

/*
	1TB = 1024GB
	1GB = 1024MB
	1MB = 1024KB
	1KB = 1024B 
*/

static void show_bit_convert(double bit_count) {
	printf("%.3f bit = %.3f KB\n", bit_count, bit_count / 1024);
	printf("%.3f bit = %.3f MB\n", bit_count, bit_count / 1024 / 1024);
	printf("%.3f bit = %.3f GB\n", bit_count, bit_count / 1024 / 1024 / 1024 );
	printf("%.3f bit = %.3f TB\n", bit_count, bit_count / 1024 / 1024 / 1024 / 1024);

}

void main() {
	double bit_count = 123456789; 
	show_bit_convert(bit_count);
}