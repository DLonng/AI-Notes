#include <stdio.h>
#include <stdlib.h>

/*
 * return: 2^num
 */
static int my_two_power_while(int num) {
	int result = 1;
	int i = 0;
	while (i < num) {
		result *= 2;
		i++;
	}
	 
	return result;
}

static int my_two_power_for(int num) {
	int result = 1;
	for (int i = 0; i < num; i++)
		result *= 2;
	
	return result;
}


static int my_two_power_do_while(int num) {
	int result = 1;
	int i = 0;
	if (0 == num)
		return 1;
	do {
		 
		 result *= 2;
		 i++;
	 } while (i < num);

	return result;
}

static int my_two_power_rec(int num) { 
	if (0 == num)
		return 1;

	static int result = 1;
	static int i = 0;
	
    if (i >= num) {
		return result;
    } else {
		result *= 2;
		i++; 
		return my_two_power_rec(num); 
    }
}

static int my_two_power_goto(int num) {
	int result = 1;
	int i = 0;

A:	 
	if (i < num) {
		 result *= 2;
		 i++;
		 goto A;
	}  

	return result;

}

int main(void) {
	printf("%d\n", my_two_power_while(5));
	printf("%d\n", my_two_power_for(5));
	printf("%d\n", my_two_power_do_while(5));
	printf("%d\n", my_two_power_rec(5));
	printf("%d\n", my_two_power_goto(5));
	return 0;
}
