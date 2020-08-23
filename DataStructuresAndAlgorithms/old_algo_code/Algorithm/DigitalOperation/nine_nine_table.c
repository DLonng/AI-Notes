#include <stdio.h>
#include <stdlib.h> 

/*
	0	1	2	3	4	5	6	7	8	9	
	1	1	
	2	2	4	
	3	3	6	9	
	4	4	8	12	16	
	5	5	10	15	20	25	
	6	6	12	18	24	30	36	
	7	7	14	21	28	35	42	49	
	8	8	16	24	32	40	48	56	64	
	9	9	18	27	36	45	54	63	72	81
*/
static void show_nine_nine_table() {
	for (int i = 0; i < 10;i++)
		printf("%d\t", i);
	
	printf("\n");
 
	for (int i = 1; i < 10;i++) {
		printf("%d\t", i);
		for (int j = 1; j <= i;j++) {
			printf("%d\t", i * j);
		}
		printf("\n");
	}
}

void main() { 
	show_nine_nine_table();
}