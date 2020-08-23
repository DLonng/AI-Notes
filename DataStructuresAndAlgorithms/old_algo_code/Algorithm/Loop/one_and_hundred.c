#include <stdio.h>
#include <stdlib.h>


static int one_add_to_hundred() {
	int i = 1;
	int sum = 0;
	while (i <= 100)
		sum += (i++);
	return sum;
}


int main(void) {
	printf("1 + 2 + ... + 100 = %d\n", one_add_to_hundred());
	return 0;
}
