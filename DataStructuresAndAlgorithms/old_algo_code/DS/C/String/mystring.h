#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
	char *pstr;
	int length;
} mystring;

int mystrlen(const char* str);

char *mystrcpy(char *dest, const char *source);

char *mystrcat(char *source, const char *add_str);

char *mystrchr(char *dest, char find_char);

char *mystrstr(const char *dest, const char *findstr);

void init(mystring *string);

void init_with_length(mystring *string, int length);

void init_with_string(mystring *string, const char *init_string);

void string_back_add_char(mystring *string, char add_ch);

void string_back_add_string(mystring *string, char *add_str);

char *string_find_first_char(mystring *string, char find_char);

char *string_find_first_str(mystring *string, char *find_str);

int del_first_find_char(mystring *string, char del_ch);

int del_first_find_str(mystring *string, char *find_str);
