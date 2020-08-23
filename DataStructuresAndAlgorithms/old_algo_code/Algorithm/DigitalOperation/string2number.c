#include <stdio.h>
#include <math.h>

int string2number(const char *str) {
	char *p = str;
	int num = 0;
	  
	// Get number digit of str.
	for (p = str; *p != '\0'; p++) {
		if (str[0] == '0')	
			return -1;
		num++;
    }

	//printf("%d\n", num);
	int result = 0;
	int tmp = 0;

	/*
	 * 12345 
	 *	1 	* 10 + 2 = 12
	 * 	12	* 10 + 3 = 123
	 *	123	* 10 + 4 = 1234
	 * 	1234* 10 + 5 = 12345
	 */
	for (int i = 0; i < num; i++) {
		result *= 10;
		// '0' = 48 => ch - 48 = num
	    tmp = str[i] - 48;
		result += wei;
	}

	return result;
}

void main() {
	printf("%d\n", string2number("12345"));
}