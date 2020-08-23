#include <stdio.h>
#include <string.h>
#include <stdlib.h>

char* my_itoa(int x) {
	//x = 0
	if (!x) { 
		char* temp = (char*)malloc(2);
		temp[0] = '0';
		temp[1] = '\0';
		return temp;
	}
	
	//x < 0
	int temp_change_x = x;
	int flag = 1;
	if (x < 0) {
		if (x != -2147483648){
			temp_change_x = -temp_change_x;
		} else {
			char* temp = (char*)malloc(12);
			strcpy(temp, "-2147483648");
			return temp;
		}
		//printf("temp_change_x = %d\n", temp_change_x);
		flag = 0;
	}

	//x > 0
	int num  = 0;
	char temp_ch = 0;
    char* ret_str = NULL;
	int temp_x = temp_change_x;
	
	//get num
	while (temp_x) {
		temp_x /= 10;
		num++;
	}

	temp_x = temp_change_x;
	ret_str = (char*)malloc(num + 1);
    for (int i = 0; i < num; i++) {
		temp_ch = temp_x % 10 + 0x30;

		ret_str[num - i - 1] = temp_ch;
		
		temp_x /= 10;
	}
	ret_str[num] = '\0';

	//if flag = 0, then we need to add '-' to the head of the ret_str
	if (!flag) {
		char* temp_ret_str = (char*)malloc(num + 2);
		temp_ret_str[0] = '-';
		
		strcat(temp_ret_str, ret_str);
		
		free(ret_str);
		
		return temp_ret_str;
	}

	return ret_str;
}


int main() {
	int x = 0;
	while(1) {
		puts("\nplease input a int number([-2147483648,2147483647]):");
		scanf("%d", &x);
		
		printf("\nx = %d\n", x);
		
		char* ret_str = my_itoa(x);
		
		printf("convert:%s\n",ret_str);
		
		//Don`t forget to free	
		free(ret_str);
	}
	return 0;
}