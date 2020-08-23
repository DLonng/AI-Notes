#include <stdio.h>
#include <stdlib.h>
/*
 * 质数定义为在大于 1 的自然数中，除了 1 和它本身以外不再有其他因数的数称为质数
 */
static void prime_num(int num) {
	if (num <= 1) {
		printf("no prime.\n");
	} else {
		for (int i = 2; i < num; i++) {
			if (num % i == 0) {
				printf("no prime.\n");
				return;
			}
		}	
		printf("is prime.\n");
	}
}

int main(int argc, char **argv) {
	prime_num(atoi(argv[1]));
	return 0;
}