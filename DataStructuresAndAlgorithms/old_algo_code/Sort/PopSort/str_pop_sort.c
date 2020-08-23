#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void show_str(char *cmd[10]) {
	for (int i = 0; i < 10; i++)
		puts(cmd[i]); 
}

static void pop_sort_normal(char *cmd[], int n){
	for (int i = 0; i < n - 1; i++) {
		for (int j = 0; j < n - 1 - i; j++) {
			if (strcmp(cmd[j], cmd[j + 1]) > 0) {
				char *temp = cmd[j];
				cmd[j] = cmd[j + 1];
				cmd[j + 1] = temp;
			}
		}
	} 
}



static void pop_sort_best(char *cmd[], int length) {
	char *temp = NULL;
	int need_sort = 1;
	for (int i = 0; (i < length) && need_sort; i++) {
		need_sort = 0;
		for (int j = length - 1; j > i; j--) {
			// < 
			if (strcmp(cmd[j - 1], cmd[j]) > 1) {
				temp = cmd[j - 1];
				cmd[j - 1] = cmd[j];
				cmd[j] = temp;
				
				// Don't forget !
				need_sort = 1;
			}
		}
	}
}

int main(void) {
	char *cmd[10] = { "calc", "1234", "abcd", "ak47", "AMP", "M16", "notepad", "tasklist", "run", "mspaint" };
	
	show_str(cmd);
	
	pop_sort_normal(cmd, 10);
	//pop_sort_best(cmd, 10);
	
	putchar('\n');
	
	show_str(cmd);
	return 0;
}
