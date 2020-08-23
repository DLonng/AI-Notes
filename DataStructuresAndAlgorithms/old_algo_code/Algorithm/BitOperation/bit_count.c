#include <stdio.h>
#include <stdlib.h>



int NumberOfSetBits(int i) {
	i = i - ((i >> 1) & 0x55555555);
	i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
	return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}


unsigned int bit_count(unsigned int value) {
	unsigned int count = 0;
	while (value > 0) {           // until all bits are zero
		if ((value & 1) == 1)     // check lower bit
			count++;
		value >>= 1;              // shift bits, removing lower bit
	}
	return count;
}

int bit_count2(int x) {
  int i = 0;
  while (x) {
	i++;
	/*
	 * x = 1111
	 * i = 1, x & (x - 1) = 1111 & 1110 = 1110 != 0
	 * i = 2, x & (x - 1) = 1110 & 1101 = 1100 != 0
	 * i = 3, x & (x - 1) = 1100 & 1011 = 1000 != 0
	 * i = 4, x & (x - 1) = 1000 & 0111 = 0000 == 0
	 */
    x = x & (x - 1);
  }

  return i;
}

static int bit_count3(int number) {
    int count = 0;
    for (int i = 0; i < 32; i++) {
        if ((number & (1 << i)) != 0) {
            count++;
        }
    }
    
    return count;
}

int main(void)
{
	printf("7 bit count: %u\n", bit_count(7));
	printf("7 bit count: %d\n", NumberOfSetBits(7));
	printf("7 bit count: %d\n", bit_count2(7));
	printf("7 bit count: %d\n", bit_count3(7));
	return 0;
}
