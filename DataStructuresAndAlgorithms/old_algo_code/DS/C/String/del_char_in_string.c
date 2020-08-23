#include <stdio.h>


static void del_char_in_string(char *string, char del_ch) {
	char *p = string;
	char *cur = NULL;
	char *next = NULL;
	
	while (*p) {
		if (*p == del_ch) {
			cur = p;
			next = p + 1;
			while (*next) {
				// Del all char that equal del_ch
				*cur = *next;
				cur++;
				next++;
			}
			// Don`t end string.
			*cur = '\0';
		} else {
			p++;
		}
	} 
}

void main() {
	char str[] = "Hello choeonogozhi";
	del_char_in_string(str, 'o');
	puts(str);
}