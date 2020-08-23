#include <stdio.h>
#include <stdlib.h>

char *number2string(int num) {
	int n = 0;
	for (int inum = num; inum; inum /= 10)
		n++;

	// Don`t forget '\0'
	char *result = (char *)malloc(sizeof(char) * n + 1);

	/*
	 * 12345
	 * result[4] = 12345 % 10 + 48 = 5 + 48 = '5', 12345 / 10 = 1234
	 * result[3] = 1234  % 10 + 48 = 4 + 48 = '4', 1234  / 10 = 123
	 * result[2] = 123   % 10 + 48 = 3 + 48 = '3', 123   / 10 = 12
	 * result[1] = 12    % 10 + 48 = 2 + 48 = '2', 12    / 10 = 1
	 * result[0] = 1     % 10 + 48 = 1 + 48 = '1', 1     / 10 = 0
	 */
	for (int i = n; num != 0; num /= 10, i--)
		result[i - 1] = num % 10 + 48;

	return result;
}

void main(int argc, char **argv) {
	char *result = number2string(atoi(argv[1]));
	printf("%s\n", result);
	free(result);
}