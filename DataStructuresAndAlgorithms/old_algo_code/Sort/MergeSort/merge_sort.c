#include <stdio.h>
#include <stdlib.h>  

/**
 * @synopsis  Merge 将有序的的 SR[i ... m] 和 SR[m + 1 ... n] 归并为有序的 TR[i ... n]
 *
 * @param SR[]
 * @param TR[]
 * @param i
 * @param m
 * @param n
 */
void Merge(int SR[], int TR[], int i, int m, int n) {
	int j = 0;
	int k = 0;
	int l = 0;

	// 1. 将 SR 中的记录从小到大归并入 TR
	for (j = m + 1, k = i; i <= m && j <= n; k++) {
		if (SR[i] < SR[j])
			TR[k] = SR[i++];
		else
			TR[k] = SR[j++];
	}
	
	// 2. 将剩余的 SR[i ... m] 复制到 TR 中
	if (i <= m){
		for (l = 0; l <= m - i; l++)
			TR[k + l] = SR[i + l];
	}
	
	// 3. 将剩余的 SR[j ... n] 复制到 TR 中
	if (j <= n){
		for (l = 0; l <= n - j; l++)
			TR[k + l] = SR[j + l];
	}
}


/**
 * @synopsis  MSort 
 *
 * @param SR[]
 * @param TR1[]
 * @param s
 * @param length
 *
 * 0. 基本思想
 * 		先递归划分子问题，然后合并结果。把待排序列看成两个有序的子序列，然后合并两个子序列，然后把子序列看成由两个有序序列。
 * 		倒着来看，其实就是先两两合并，然后四四合并，最终形成有序序列。
 * 1. 空间复杂度
 * 		需要与原始记录序列同样数量的存储空间：O(n)
 * 2. 时间复杂度
 * 		最好，最坏，平均：O(nlogn)
 * 3. 优缺点
 * 		比较占用内存，但是效率高，稳定
 * 4. 适用场景
 * 		排序数量比较大，并且要求稳定，常用于外部排序（多路归并排序）
 */
void MSort(int SR[], int TR1[], int s, int length) {
	int m;
	int TR2[100];

	if (s == length) {
		TR1[s] = SR[s];
	} else {
		m = (s + length) / 2;			// 从序列的中间下标开始拆分
		MSort(SR, TR2, s, m);			// 拆分左序列
		MSort(SR, TR2, m + 1, length);	// 拆分右序列
		Merge(TR2, TR1, s, m, length);	// 左右序列归并
	}
}

void MergeSort(int* L, int length) {
	MSort(L, L, 0, length - 1); //数组的最后一个元素不存在，所以减 1
}

// 非递归归并排序 - 没有弄明白
void MergePass(int SR[], int TR[], int s, int n) {
	int i = 0;//数组从0开始
	int j = 0;

	while (i <= n - 2 * s - 1) {
		Merge(SR, TR, i, i + s - 1, i + 2 * s - 1);
		i = i + 2 * s;//加上增量
	}

	if (i < n - s + 1) {
		Merge(SR, TR, i, i + s - 1, n);
	} else {
		for (j = i; j <= n; j++)
			TR[j] = SR[j];
	}
}

void MergerSort_rec(int* L, int length) {
	int* TR = (int*)malloc(sizeof(int) * length);
	int k = 1;
	
	while (k < length) {
		MergePass(L, TR, k, length);
		k = 2 * k;
		MergePass(TR, L, k, length);
		k = 2 * k;
	}
}

void MergerSort_rec(int* L, int length) {
	MergerSort_rec(L, length - 1);//数组最后一个元素不存在
}


int main(void) {
	int x[10] = { 50, 10, 90, 30, 70, 40, 80, 60, 20, 100 };
	
	MergeSort(x, 10);
	
	for (int i = 0; i < 10; i++)
		printf("%d ", x[i]);
	printf("\n");
	return 0;
}
