#include "mystring.h"


int mystrlen(const char* str) {
	if ('\0' == *str) {
		return 0;
	} else {
		return mystrlen(++str) + 1;
	}
}

char *mystrcpy(char *dest, const char *source) {
	if (dest == NULL || source == NULL)
		return NULL;
	
	char * destbak = dest;
	
	while (*source)
		*dest++ = *source++;

	*dest = '\0';
	return destbak;
}


char *mystrcat(char *source, const char *add_str) {
	if (source == NULL || add_str == NULL) {
		return NULL;
	} else {
		char *source_bak = source;
		
		while (*source)
			source++;
		
		while (*add_str)
			*source++ = *add_str++;
		
		*source = '\0';
		return source_bak;
	}
}


char *mystrchr(char *dest, char find_char) {
	if (dest == NULL)
		return NULL;
	
	char *dest_bak = dest;

	while (*dest_bak) {
		if (*dest_bak == find_char)
			return dest_bak;
		dest_bak++;
	}

	return NULL;
}



char *mystrstr(const char *dest, const char *findstr){
	if (dest == NULL || findstr == NULL)
		return NULL;
	
	char *destbak = dest;
	char *p = NULL;

	while (*destbak) {
		// flag = 1 -> find, otherwise no find.
		int flag = 1;
		
		char *findstrbak = findstr;
		char *nowdestbak = destbak;
		
		while (*findstrbak) {
			if (*nowdestbak) {
				if (*findstrbak != *nowdestbak)
					flag = 0;
				nowdestbak++;
				findstrbak++;
			} else {
				flag = 0;
				break;
			}
		}

		if (flag) {
			p = destbak;
			return p;
		}
		

		destbak++;
	}

	return NULL;
}



void init(mystring *string) {
	string->pstr = NULL;
	string->length = 0;
}


void init_with_length(mystring *string, int length) {
	string->pstr = (char *)calloc(length, sizeof(char));
	string->length = length;
}


void init_with_string(mystring *string, const char *init_string) {
	int length = mystrlen(init_string);
	string->pstr = (char *)calloc(length + 1, sizeof(char));
	mystrcpy(string->pstr, init_string);
	string->length = length + 1;
}


void string_back_add_char(mystring *string, char add_ch) {
	if (mystrlen(string->pstr) + 1 == string->length) {
		string->pstr = realloc(string->pstr, string->length + 1);
		string->length += 1;
		string->pstr[string->length - 2] = add_ch;
		string->pstr[string->length - 1] = '\0';
	} else {
		int cur_length = mystrlen(string->pstr);
		string->pstr[cur_length] = add_ch;
		string->pstr[cur_length + 1] = '\0';
	}
}

void string_back_add_string(mystring *string, char *add_str) {
	int cur_length = mystrlen(string->pstr);
	int add_length = mystrlen(add_str);

	if (cur_length + add_length + 1 > string->length) {
		int need_add_length = cur_length + add_length + 1 - (string->length);
 		string->pstr = (char *)realloc(string->pstr, string->length + need_add_length);
		mystrcat(string->pstr, add_str);
		string->length += need_add_length;
	} else {
		mystrcat(string->pstr, add_str);
	}
}

char *string_find_first_char(mystring *string, char find_char) {
	char *first_find_char = mystrchr(string->pstr, find_char);
	return first_find_char;
}


char *string_find_first_str(mystring *string, char *find_str) {
	return mystrstr(string->pstr, find_str);
}


int del_first_find_char(mystring *string, char del_ch) {
	char *pdelchar = mystrchr(string->pstr, del_ch);
	if (NULL == pdelchar) {
		return 0;
	} else {
		char *pnext = pdelchar + 1;
		while (*pnext) 
			*pdelchar++ = *pnext++;
		*pdelchar = '\0';
		return 1;
	}
}

int del_first_find_str(mystring *string, char *find_str) {
	char *pfindstr = mystrstr(string->pstr, find_str);
	if (NULL == pfindstr) {
		return 0;
	} else {
		int length = mystrlen(find_str);
		char *pnext = pfindstr + length;
		while (*pnext) 
			*pfindstr++ = *pnext++;
		*pfindstr = '\0';
		return 1;
	}
}


int main() {

	return 0;
}
