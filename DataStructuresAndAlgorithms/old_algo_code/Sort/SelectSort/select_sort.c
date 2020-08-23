#include <stdio.h>
#include <stdlib.h>
#include <time.h>


static void swap(int *data1, int *data2) {
	if (NULL == data1) {
		printf("data1 = NULL\n");
		return;
	}

	if (NULL == data2) {
		printf("data2 = NULL\n");
		return;
	}

	int tmp = *data1;
	*data1 = *data2;
	*data2 = tmp;
}




/**
 * @synopsis  select_sort 
 *
 * @param a[]
 * @param n
 *
 * 0. 基本思想：
 * 		将一个数组分成有序区和无序区，初始时整个数组都是无序区，
 * 		然后每次从无序区选一个最小的元素直接放到有序区的最后，直到整个数组变有序区
 * 
 * 1. 时间复杂度
 *		平均 O(n^2)
 *		最坏 O(n^2)
 *		最好 O(N^2)
 * 
 * 2. 空间复杂度
 * 		O(1)
 *
 * 3. 稳定性
 * 		不稳定
 *
 * 4. 复杂性
 *		简单
 * 
 * 5. 适用场合
 *		适合小规模的数据排序		
 */
static void select_sort(int a[], int n) {
	// 1. 判断输入参数是否合法
	if (NULL == a) {
		printf("a = NULL\n");
		return;
	}

	if (n < 1) {
		printf("n < 1\n");
		return;
	}
	
	// 2. 从第一个位置开始
	for (int i = 0; i < n - 1; i++) {
		int min_i = i;
		
		// 3. 每次寻找一个最小的索引
		for (int j = i + 1; j < n; j++) {
			if (a[j] < a[min_i])
				min_i = j;
		}
		
		// 4. 最小索引变化则交换
		if (min_i != i) {
			/*
			a[min_i] = a[min_i] ^ a[i];
			a[i] = a[i] ^ a[min_i];
			a[min_i] = a[min_i] ^ a[i];
			*/
			swap(&a[min_i], &a[i]);
		}
	}
}


void main() {
	time_t ts;
	srand((unsigned int)time(&ts));
	int a[10] = { 0 }; 
	for (int i = 0; i < 10;i++) {
		a[i] = rand() % 100;
		printf("%d\t%x\n", a[i], &a[i]);
	} 

	
	printf("\n\n");
	// select_sort(a, sizeof(a) / sizeof(a[0]));
	// select_sort(NULL, sizeof(a) / sizeof(a[0]));
	select_sort(a, 0);

	printf("select_sort:\n\n");
	for (int i = 0; i < 10; i++)
		printf("%d\t%x\n", a[i], &a[i]);
	
	 
}
