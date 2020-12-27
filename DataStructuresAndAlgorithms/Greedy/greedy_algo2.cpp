#include <cmath>
#include <iostream>

/**
  * @brief 函数简要说明
  * @details 函数细节说明
  * @param 参数名 [in] 参数描述
  * @param 参数名 [out] 参数描述
  * @return pointer to the updated NODE
  * @note 注意事项
  * @todo 需要去实现的功能
  * @author 作者
  * @date 日期
  */

////////////////////////////////////////////组合最大的数////////////////////////////////////////////

/**
  * @brief determine which combination value is the largest
  * @details if num1 = 23, num2 = 123, then num1_num2 = 23123 > num2_num1 = 12323 
  * 
  * @param num1 [in]
  * @param num2 [in]
  * @return if num1_num2 > num2_num1, return true, otherwise false
  * 
  * @author DLonng
  * @date 2020-12-27
  */
bool MaxCombinationNum(int num1, int num2)
{
    // num1 和 num2 的位数
    int count1 = 0;
    int count2 = 0;

    // 保存 2 个数的副本用来计算位数
    int tmp_num1 = num1;
    int tmp_num2 = num2;

    // 计算 num1 的位数
    while (tmp_num1) {
        ++count1;
        tmp_num1 /= 10;
    }

    // 计算 num2 的位数
    while (tmp_num2) {
        ++count2;
        tmp_num2 /= 10;
    }

    // num1 = 23, num2 = 123
    // num1_num2 = 23 * 10^3 + 123 = 23123
    // num2_num1 = 123 * 10^2 + 23 = 12323
    int num1_num2 = num1 * pow(10, count2) + num2;
    int num2_num1 = num2 * pow(10, count1) + num1;

    return (num1_num2 > num2_num1) ? true : false;
}

/**
  * @brief find the largest number composed of all the values in the nums array using pop sort
  * 
  * @param nums [out] the number array
  *             such as: { 12, 43, 456, 76, 23 }
  *             result : { 76, 456, 43, 23, 12 }
  * @param n [in] the length of nums array
  * 
  * @author DLonng
  * @date 2020-12-27
  */
void GreedyNumCombination(int nums[], int n)
{
    int temp = 0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n - i - 1; j++) {
            if (MaxCombinationNum(nums[j], nums[j + 1])) {
                temp = nums[j];
                nums[j] = nums[j + 1];
                nums[j + 1] = temp;
            }
        }
    }
}

int main_greedy_num_combination(int argc, char* argv[])
{
    int N;

    std::cout << "please enter the number of N:" << std::endl;
    std::cin >> N;

    std::cout << "please enter each number with space:" << std::endl;

    int* array = new int[N];
    for (int i = 0; i < N; i++)
        std::cin >> array[i];

    // 使用贪心策略对数组进行冒泡排序，每次对相邻的 2 个数进行前后组合数值排序
    GreedyNumCombination(array, N);

    std::cout << "max number is:";
    for (int i = N - 1; i >= 0; --i)
        std::cout << array[i];

    std::cout << std::endl;

    delete[] array;

    return 0;
}

////////////////////////////////////////////找零钱问题////////////////////////////////////////////

/**
  * @brief 找零钱问题，人民币纸币的面值有 1 元 5 元 10 元 20 元 50 元 100 元
  *        要求设计一个程序，输入找零的钱，输出找钱方案中所需最少纸币张数的方案
  * @details 比如 456 元，最少是 4 张 100，1 张 50，1 张 5 元，1 张 1 元，一共 7 张
  * 
  * @param exchange_money [in] 要找零的钱
  * @param money_class [in] 纸币面值数组，面额从大到小排序 { 100, 50, 20, 10, 5, 1 }
  * @param money_nums [out] 返回每种纸币的需要的数量，面额对应与 money_class 数组
  * @param monry_n [in] 纸币类别，等于面值数组长度
  * @param total_money_nums [out] 总共需要的最少纸币数量
  * 
  * @todo int[] 换成 C++ vector
  * 
  * @author DLonng
  * @date 2020-12-27
  */
void GreedMoney(int exchange_money, int money_class[], int money_nums[], int money_n, int& total_money_nums)
{
    // 当前已经找零的钱
    int current_money = 0;

    for (int i = 0; i < money_n;) {
        if (current_money + money_class[i] > exchange_money) {
            // 当前面额的钱加上当前总钱数 > 要找零的钱，就选择面额较小的纸币
            // 400 + 100 > 456
            // 选择 400 + 50 < 456
            i++;
            continue;
        }

        // 把当前面额的零钱累加起来
        current_money += money_class[i];

        // 把当前累加的零钱数量 + 1
        money_nums[i]++;

        // 累加总的零钱数量
        total_money_nums++;

        // 找零完毕
        if (current_money == exchange_money)
            break;
    }
}

int main_greedy_money(int argc, char* argv[])
{
    const int money_n = 6;

    // 记录钱的面值
    int money_class[money_n] = { 100, 50, 20, 10, 5, 1 };
    // 记录每种面值的数量
    int money_nums[money_n] = { 0 };

    // 当前要找零的钱
    int exchange_money = 0;

    // 当前已经找零的纸币数量
    int total_money_nums = 0;

    std::cout << "please enter the all money you want to exchange:" << std::endl;
    std::cin >> exchange_money;
    
    GreedMoney(exchange_money, money_class, money_nums, money_n, total_money_nums);

    // 输出每种纸币的数量
    for (int i = 0; i < 6; ++i) {
        if (money_nums[i] != 0) {
            switch (i) {
            case 0:
                std::cout << "the 100 yuan have: " << money_nums[i] << std::endl;
                break;
            case 1:
                std::cout << "the 50 yuan have: " << money_nums[i] << std::endl;
                break;
            case 2:
                std::cout << "the 20 yuan have: " << money_nums[i] << std::endl;
                break;
            case 3:
                std::cout << "the 10 yuan have: " << money_nums[i] << std::endl;
                break;
            case 4:
                std::cout << "the 5 yuan have: " << money_nums[i] << std::endl;
                break;
            case 5:
                std::cout << "the 1 yuan have: " << money_nums[i] << std::endl;
                break;
            }
        }
    }

    // 输出总的零钱纸币数量
    std::cout << "the total money nums are:" << total_money_nums << std::endl;

    return 0;
}