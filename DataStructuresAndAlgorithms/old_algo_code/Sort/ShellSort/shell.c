#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/**
 * @synopsis  shell_sort 
 *
 * 基本思想
 * 		先将整个待排记录序列分割成为若干子序列分别进行直接插入排序，待整个序列中的记录基本有序时再对全体记录进行一次直接插入排序
 * 
 * 时间复杂度
 * 		平均：O(n^1.3)
 * 
 * 空间复杂度
 * 		O(1)
 * 
 * 稳定性
 * 		不稳定
 *
 * 注意：
 * 		增量的选择比较关键，研究表明当增量序列为 2^(t - k + 1) - 1 (0 <= k <= [log2(n + 1)]) 时，可以获得不错的效果
 */
void shell_sort(int *L, int length, int t) {
	int i;
	int j;
	int temp = 0;
	int increment = 0;
	for (int k = 0; k <= t; k++) {
		// a good increment
		increment = (int)(pow(2, t - k + 1) - 1);
		// insert sort
		for (i = increment; i < length; i++) {
			if (L[i] < L[i - increment]) {
				temp = L[i];
				for (j = i - increment; j >= 0 && temp < L[j]; j -= increment)
					L[j + increment] = L[j];
				L[j + increment] = temp;
			}
		}
	}
}



int main(void) {
	int x[10] = { 5, 3, 2, 1, 7, 4, 6, 9, 8, 10 };
	shell_sort(x, 10, (int)(log(10 + 1) / log(2)));

	for (int i = 0; i < 10; i++)
		printf("%d ", x[i]);
	printf("\n");
	return 0;
}

