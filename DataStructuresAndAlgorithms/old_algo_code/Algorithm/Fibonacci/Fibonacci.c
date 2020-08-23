#include <stdio.h>
#include <stdlib.h>



int tt1(int n) {
	if (n == 1)
		return 1;
	else if (n == 2)
		return 2;
	else
		return tt1(n - 1) + tt1(n - 2);
}


int tt2(int n) {
	int a[50];
	a[0] = 1;
	a[1] = 2;
	for (int i = 2; i < n;i++)	
		a[i] = a[i - 1] + a[i - 2];
	
	return a[n - 1];
}


int tt3(int n) {
	int n1 = 1;
	int n2 = 2;
	int n3;
	for (int i = 2; i < n; i++) {
		n3 = n1 + n2;
		n1 = n2;
		n2 = n3;
	}
 
	return n3;
}


void main()
{
	printf("%d\n", tt1(5));
	printf("%d\n", tt2(5));
	printf("%d\n", tt3(5));
}