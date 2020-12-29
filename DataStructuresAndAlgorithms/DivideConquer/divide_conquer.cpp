#include <iostream>
#include <vector>

/////////////////////////////////////////////二分查找问题/////////////////////////////////////////////
int k;

int BinarySearch(int a[], int x, int low, int high)
{
    if (low > high) {
        return -1;
    }

    int mid = (low + high) / 2;

    if (x == a[mid]) {
        //k = mid;
        return x;
    } else if (x > a[mid]) {
        return BinarySearch(a, x, mid + 1, high);
    } else {
        return BinarySearch(a, x, low, mid - 1);
    }
}

int main_bs()
{
    int a[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    printf("请输入需要查找的正数字：");

    int x;
    scanf("%d", &x);

    int r = BinarySearch(a, x, 0, 9);

    if (r == -1) {
        printf("没有查到");
    } else {
        printf("查到了, n = %d", r);
    }

    return 0;
}

/////////////////////////////////////////////全排列问题/////////////////////////////////////////////

void swap(int arr[], int i, int j)
{
    int tmp = arr[i];
    arr[i] = arr[j];
    arr[j] = tmp;
}

// ????
void FullSort(int arr[], int n, int start, int end)
{
    if (start == end) {
        for (int i = 0; i < n; i++) {
            std::cout << i << "";
        }

        return;
    }

    for (int i = start; i <= end; i++) {
        swap(arr, i, start);
        FullSort(arr, n, start + 1, end);
        swap(arr, i, start);
    }
}

int main_full_sort()
{
    int arr[] = { 1, 2, 3, 4 };

    FullSort(arr, 4, 0, 4 - 1);

    return 0;
}

/////////////////////////////////////////////归并排序问题/////////////////////////////////////////////

// 把 2 个有序子序列 [left, mid], [mid + 1, right] 合并
void Merge(int a[], int n, int left, int mid, int right)
{
    int tmp[n];

    // p1、p2 是检测指针，k 是存放指针
    int p1 = left;
    int p2 = mid + 1;
    int k = left;

    while ((p1 <= mid) && (p2 <= right)) {
        if (a[p1] <= a[p2])
            tmp[k++] = a[p1++];
        else
            tmp[k++] = a[p2++];
    }

    while (p1 <= mid)
        tmp[k++] = a[p1++];

    while (p2 <= right)
        tmp[k++] = a[p2++];

    for (int i = left; i <= right; i++)
        a[i] = tmp[i];
}

void MergeSort(int a[], int n, int start, int end)
{
    // 当子序列中只有一个元素时结束递归
    if (start < end) {
        // 折半划分
        int mid = (start + end) / 2;

        // 递归排序左边子序列
        MergeSort(a, n, start, mid);

        // 递归排序右边子序列
        MergeSort(a, n, mid + 1, end);

        // 合并子问题的解
        Merge(a, n, start, mid, end);
    }
}

int main_merge()
{
    int a[10] = { 2, 1, 3, 5, 4, 10, 7, 9, 8, 6 };

    for (int i = 0; i < 10; i++)
        std::cout << a[i] << " ";

    std::cout << std::endl;

    MergeSort(a, 10, 0, 9);

    for (int i = 0; i < 10; i++)
        std::cout << a[i] << " ";

    std::cout << std::endl;

    return 0;
}

/////////////////////////////////////////////最大连续和问题/////////////////////////////////////////////

int MaxContinuitySum(int a[], int l, int r)
{
    if (l == r)
        return a[l];

    int m = (l + r) / 2;

    //（分解）情况 1：完全在左区间，或者完全在右区间
    int max_sum = std::max(MaxContinuitySum(a, l, m), MaxContinuitySum(a, m + 1, r));

    //（合并）情况 2：横跨左右两个区间
    int sum_l = a[m];
    int t = 0;

    for (int i = m; i >= l; i--)
        sum_l = std::max(sum_l, t += a[i]);

    int sum_r = a[m + 1];
    t = 0;

    for (int i = m + 1; i <= r; i++)
        sum_r = std::max(sum_r, t += a[i]);

    return std::max(max_sum, sum_l + sum_r);
}

void Move(int m, char x, char y, char z)
{
    if (m == 1) {
        // 如果只有一个金片了，那么我们直接将其从 X 移至 Z 即可
        std::cout << x << " -> " << z << std::endl;
        return;
    }

    // 先将 m - 1 个金片从 X 移至 Y
    Move(m - 1, x, z, y);

    // 每次递归时，我们总是将 1 号金片移至 Z
    std::cout << x << " -> " << z << std::endl;

    // 再将整体代换的 m - 1 个金片从 Y 移至 Z
    Move(m - 1, y, x, z);
}

int main()
{
    int m;
    
    std::cin >> m;

    Move(m, 'A', 'B', 'C');
}
