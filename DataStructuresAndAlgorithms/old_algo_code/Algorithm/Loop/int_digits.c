#include <stdio.h>

/*
 * Count num digits.
 * eg: 
 * 		123456 -> 6
 * 		123 -> 3
 */
int count_while(int num) {
	int i = 0;
	while (num) {
		num /= 10;
		i++;
	}

	return i; 
}


int count_do_while(int num) {
	int i = 0;
	do {
		num /= 10;
		i++;
		 
	} while (num);
	 
	return i;
}

int count_for(int num) {
	int i = 0;
	for (i = 0; num; i++)
		num /= 10;
	
	return i; 
} 

int count_goto(int num) {
	int i = 0;
A:
	if (num) {
		num /= 10;
		i++;
		goto A;
	}

	return i;
 }

int count_rec(int num) {
	static int i = 0;
	if (0 == num) {
		return i;
	} else {
		num /= 10;
		i++;
		return count_rec(num);
	} 
}

int main(void) {
	printf("%d\n", count_while(123456));
	printf("%d\n", count_do_while(123456));
	printf("%d\n", count_for(123456));
	printf("%d\n", count_goto(123456));
	printf("%d\n", count_rec(123456));
	 
	return 0;
}