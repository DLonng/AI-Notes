#include <iostream>
#include <cmath>

using namespace std;

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


// 贪心算法学习：https://blog.csdn.net/effective_coder/article/details/8736718


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


/////////////////////////////////////////找零钱问题（这种代码不简洁，不要用）///////////////////////////////////////////////////
#define ONEFEN 1
#define FIVEFEN 5
#define TENFEN 10
#define TWENTYFINEFEN 25

int main1()
{
    int sum_money = 41;
    int num_25 = 0, num_10 = 0, num_5 = 0, num_1 = 0;

    // 41 - 25 = 16
    while (sum_money >= TWENTYFINEFEN) {
        num_25++;
        sum_money -= TWENTYFINEFEN;
    }

    // 16 - 10 = 6
    while (sum_money >= TENFEN) {
        num_10++;
        sum_money -= TENFEN;
    }

    // 6 - 5 = 1
    while (sum_money >= FIVEFEN) {
        num_5++;
        sum_money -= FIVEFEN;
    }

    // 1 - 1 = 0
    while (sum_money >= ONEFEN) {
        num_1++;
        sum_money -= ONEFEN;
    }

    //输出结果
    cout << "25 分硬币数：" << num_25 << endl;
    cout << "10 分硬币数：" << num_10 << endl;
    cout << "5 分硬币数：" << num_5 << endl;
    cout << "1 分硬币数：" << num_1 << endl;

    return 0;
}

////////////////////////////////////////找零钱问题///////////////////////////////////////////////////

/**
 * @brief 找零钱问题，使得所用的纸币数量最少，如 456 元可以最少分为多少张零钱
 * 
 * @param money 要计算的钱数
 * @param n 纸币面额的类别
 * @param value 每种纸币的面额，从小到大排序
 * @param count 每种纸币的张数，对应 value 的面额
 */
void SolveMoney(int money, int n, int value[], int count[], int num[])
{
    // 临时保存每种面额纸币的数量
    int c = 0;

    // 分别计算每种纸币需要的多少张，从面额最高的开始
    for (int i = n - 1; i >= 0; i--) {
        // min(456 / 100, 5) = min(4, 5) = 4
        // 不能超过每种面额纸币的最大数量
        c = std::min(money / value[i], count[i]);

        // 计算剩余的钱数
        // 456 = 456 - 4 * 100 = 56
        money = money - c * value[i];

        // 累加需要的纸币数量
        num[i] += c;
    }

    // 不能分解的钱数，返回 -1
    //if (money > 0)
    //    num = -1;
}

int main2()
{
    int money = 456;

    //每一张纸币的数量
    const int N = 5;
    int count[N] = { 5, 2, 2, 3, 5 };
    int value[N] = { 1, 5, 10, 50, 100 };

    // 保存每种纸币面额需要的数量
    int num[N] = { 0 };

    SolveMoney(money, N, value, count, num);

    // 纸币面额：1    5   10  50  100
    // 对应数量：1    1   0   1   4
    for (int i = 0; i < N; i++)
        std::cout << value[i] << " 元纸币需要: " << num[i] << " 张" << std::endl;

    return 0;
}

//////////////////////////////////////////背包问题////////////////////////////////////////////////////

// 装入背包中的物品定义
struct Node {
    // 字符表示，从 A 开始
    char char_mark;
    // 重量
    float weight;
    // 价值
    float value;
    // 单位重量的价值 = value / weight
    float pre_value;
    // 是否装进背包中
    bool has_in_pkg;
};

/**
 * @brief 有一个背包，容量是 M = 150，有 7 个物品，每个物品可以分割成任意大小，要求尽可能让装入背包中的物品总价值最大，但不能超过总容量。
 * 
 * @details 物品 A  B  C  D  E  F  G
 *          重量 35 30 60 50 40 10 25
 *          价值 10 40 30 50 35 40 30
 *          每次选取单位重量价值最大的物品，成为解本题的策略。
 * 
 * @todo 建议把数组换成 C++ vector
 */
int GreedyBag(float weights[], float values[], Node items[], char pkg_items[], float pkg_m, float& weight_all, float& value_max)
{
    // 保存最大单位价值的物品
    float max_pre_value = 0.0;

    //vector<Node> pkg_items;

    // 保存最大单位价值的物品索引
    int max_pre_value_index = -1;
    // 保存装入背包中的物品总数，实际 < M 的是物品是前 n - 1 个！
    int n = 0;

    // 当背包总重量 <= pkg_m 时，不断进行贪心策略
    while (weight_all <= pkg_m) {

        // 每次都选择单位价值最大的物品装入背包中，并且物品没有装入背包中
        for (int index = 0; index < 7; ++index) {
            if ((items[index].pre_value > max_pre_value) && (items[index].has_in_pkg == false)) {
                max_pre_value = items[index].pre_value;
                max_pre_value_index = index;
            }
        }

        // 把最大单位价值的物品装入背包中
        pkg_items[n++] = items[max_pre_value_index].char_mark;
        // 把装入背包中的物品标志位设置为 true，防止后续被重复选择
        items[max_pre_value_index].has_in_pkg = true;

        // 累加背包中的物品总重量
        weight_all += items[max_pre_value_index].weight;
        // 累加背包中的物品总价值
        value_max += items[max_pre_value_index].value;

        // 每次贪心完后，把最大单位价值的物品设置为 0，准备下一次选择
        max_pre_value = 0.0;
    }

    // 减去最后一次装入背包中的物品数量？这里认为每个物品是不可拆分的
    // 因为最后一个物品装入背包中使得背包重量超过 M，所以不能把最后一个物品重量算入总重量
    weight_all -= items[n - 1].weight;
    value_max -= items[n - 1].value;

    // 加入每个物品是可以拆分的？处理最后一个超过背包重量的物品
    // 使用动态规划

    // 返回背包中装入的物品数量
    return n - 1;
}

int main_greedy_bag(int argc, char* argv[])
{
    // 每个物品的重量
    float weights[7] = { 35, 30, 60, 50, 40, 15, 20 };
    // 每个物品的价值
    float values[7] = { 10, 40, 30, 50, 35, 40, 30 };
    // 存储 7 个物品的节点数组
    Node items[7];

    // 对 7 个物品节点初始化
    for (int i = 0; i < 7; i++) {
        items[i].char_mark = 'A' + i;
        items[i].weight = weights[i];
        items[i].value = values[i];
        items[i].pre_value = values[i] / weights[i];
        items[i].has_in_pkg = false;
    }

    // 输出每个物品的单位价值
    for (int i = 0; i < 7; i++)
        cout << items[i].char_mark << " pre_value: " << items[i].pre_value << endl;

    cout << endl;

    char pkg_items[7];
    float pkg_m = 150;
    float weight_all = 0.0;
    float value_max = 0.0;

    // 贪心策略：每次装入单位价值最大的物品
    int bag_items_num = GreedyBag(weights, values, items, pkg_items, pkg_m, weight_all, value_max);

    // 输出贪心算法选择的装入背包中的物品字符
    cout << "pkg items are: ";
    for (int i = 0; i < bag_items_num; i++)
        cout << pkg_items[i] << " ";

    cout << endl;
    cout << "weight_all: " << weight_all << endl;
    cout << "value_all: " << value_max << endl;

    return 0;
}

//////////////////////////////////////////活动时间安排问题///////////////////////////////////////////////////

/**
 * @brief 设有 N 个活动时间集合，每个活动都要使用同一个资源，比如说会议场，而且同一时间内只能有一个活动使用，
 *        每个活动都有一个使用活动的开始 si 和结束时间 fi，即他的使用区间为（si,fi）, 现在要求你分配活动占用时间表，
 *        即哪些活动占用该会议室，哪些不占用，使得他们不冲突，要求是尽可能多的使参加的活动最大化，即所占时间区间最大化！
 * 
 * @details 
 *        https://blog.csdn.net/effective_coder/article/details/8736718
 * 
 * @param activity_num 打算使用会议室的活动数量
 * @param s_time 每个活动使用会议室的开始时刻
 * @param f_time 每个活动使用会议室的结束时刻
 * @param activity_mark 保存使用会议室时间段不会重复的所有活动的编号
 * 
 * @todo 建议把数组换成 C++ vector
 */
void GreedyActivity(int activity_num, int s_time[], int f_time[], bool activity_mark[])
{
    // 从第一个活动开始，安排第一个活动占用会议室
    activity_mark[0] = true;
    // 上一个被安排的活动索引
    int pre_activity = 0;

    // 从第二个活动开始，判断当前活动的开始时间是否大于等于上一个活动的结束时间
    // 如果大于等于则可以安排当前活动继续占用会议室，否则两个活动的时间段有重合，不安排当前活动
    for (int cur_activity = 1; cur_activity < activity_num; cur_activity++) {
        if (s_time[cur_activity] >= f_time[pre_activity]) {
            activity_mark[cur_activity] = true;
            pre_activity = cur_activity;
        }
    }
}

int main_greedy_activity(int argc, char* argv[])
{
    // 每个活动的使用会议室的起始和结束时刻
    // 应该先按照起始时刻排序吧？
    int s_time[11] = { 1, 3, 0, 5, 3, 5, 6, 8, 8, 2, 12 };
    int f_time[11] = { 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };

    // 保存使用会议室的活动编号
    bool activity_mark[11] = { false };

    // 计算时间段不重复的活动编号
    GreedyActivity(11, s_time, f_time, activity_mark);

    cout << "使用会议室时间段不会重复的所有活动编号为：";
    for (int i = 0; i < 11; i++) {
        if (activity_mark[i])
            cout << i << " ";
    }

    return 0;
}

//////////////////////////////////////////线段区间覆盖问题///////////////////////////////////////////////////

/**
 * @brief 在一维空间中告诉你 N 条线段的起始坐标与终止坐标，求出这些线段一共覆盖了多大的长度
 * 
 * @details 
 *        https://blog.csdn.net/effective_coder/article/details/8736718
 * 
 * @param line_starts 线段区间起始坐标
 * @param line_ends 线段区间结束坐标
 * 
 * @todo 建议把数组换成 C++ vector
 */
int GreedyLine(int line_starts[], int line_ends[])
{
    // 初始线段总长为第一个区间的长度
    int total_len = line_ends[0] - line_starts[0];

    int pre = 0;

    for (int cur = 1; cur < 10; cur++) {
        if (line_starts[cur] >= line_ends[pre]) {
            // 当前线段起点大于上一线段终点，直接叠加当前线段长度即可
            total_len += (line_ends[cur] - line_starts[cur]);
            pre = cur;
        } else {
            if (line_ends[cur] <= line_ends[pre]) {
                // 当前线段区间终点小于上一个线段的区间终点，则这两个线段重合了，不需要累加长度，所以计算下一个区间即可
                continue;
            } else {
                // 当前线段的起点小于上一个线段的终点，但是当前线段的终点大于上一个线段的终点
                // 说明 2 个线段中间有部分重合，只需要累加当前线段与上一线段后面的未重合部分长度即可
                // 用当前线段的终点 - 上一个线段的终点
                total_len += line_ends[cur] - line_ends[pre];
                pre = cur;
            }
        }
    }

    return total_len;
}

int main_greedy_line(int argc, char* argv[])
{
    int line_starts[10] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
    int line_ends[10] = { 3, 5, 7, 6, 9, 8, 12, 10, 13, 15 };

    // 计算线段区间覆盖的总长度
    int total_len = GreedyLine(line_starts, line_ends);

    cout << "total line len is: "<< total_len << endl;

    return 0;
}