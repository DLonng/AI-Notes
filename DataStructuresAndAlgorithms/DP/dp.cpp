#include <iostream>
#include <limits>
#include <vector>

/////////////////////////////////////////////////////////////// 动态规划学习 - Fib 数列 ///////////////////////////////////////////////////////////////
// 存在重叠子问题，时间复杂度 = 递归树节点个数 2^n * 每个节点的执行时间 O(1) = O(2^n) 指数级别！
int Fib(int n)
{
    if (n == 1 || n == 2)
        return 1;

    return Fib(n - 1) + Fib(n - 2);
}

int FibHelper(std::vector<int>& memo, int n)
{
    // base case
    if (n == 1 || n == 2)
        return 1;

    // 已经计算过的从备忘录中拿出来
    if (memo[n] != 0)
        return memo[n];

    memo[n] = FibHelper(memo, n - 1) + FibHelper(memo, n - 2);

    return memo[n];
}

// 使用备忘录数组的自顶向下的递归解法
// 最多产生 n 个递归子节点，每个节点的执行时间为 O(1)
// 所以时间复杂度为 O(n * 1) = O(n)
int FibMemo(int n)
{
    if (n < 1)
        return 0;

    // 备忘录全初始化为 0
    // 因为要存储 memo[n], 所以多分配 1 个空间
    std::vector<int> memo(n + 1, 0);

    // 进行带备忘录的递归
    return FibHelper(memo, n);
}

// 用 dp table 完成自底向上的解法
// 时间复杂度为 O(n)，空间复杂度也为 O(n)
int FibDPTable(int n)
{
    if (n < 1)
        return 0;

    if (n == 1 || n == 2)
        return 1;

    // dp[0] 空着不用
    // dp[n] 存储最终结果
    std::vector<int> dp(n + 1, 0);

    // base case
    dp[1] = 1;
    dp[2] = 1;

    for (int i = 3; i <= n; i++)
        dp[i] = dp[i - 1] + dp[i - 2];

    return dp[n];
}

// 对 dp table 进行状态压缩
// 时间复杂度为 O(n)，空间复杂度压缩后为 O(1)
int FibDPCompression(int n)
{
    if (n < 1)
        return 0;

    if (n == 1 || n == 2)
        return 1;

    // 当前状态只与之前 2 个状态有关，所以将 dp table 压缩成 2 个单独的状态
    int prev = 1;
    int curr = 1;

    for (int i = 3; i <= n; i++) {
        int sum = prev + curr;
        prev = curr;
        curr = sum;
    }

    return curr;
}

int main1()
{
    std::cout << "Fib(10) = " << Fib(10) << std::endl;

    std::cout << "FibMemo(10) = " << FibMemo(10) << std::endl;

    std::cout << "FibDPTable(10) = " << FibDPTable(10) << std::endl;

    std::cout << "FibDPCompression(10) = " << FibDPCompression(10) << std::endl;

    return 0;
}

/////////////////////////////////////////////////////////////// 动态规划学习 - 找零钱 ///////////////////////////////////////////////////////////////

// 暴力递归，不处理重复子节点
int CoinChange(std::vector<int>& coins, int amount)
{
    if (amount < 0)
        return -1;

    if (amount == 0)
        return 0;

    int res = 100000;

    for (auto coin : coins) {
        int sub_problem = CoinChange(coins, amount - coin);

        if (sub_problem == -1)
            continue;

        res = std::min(res, 1 + sub_problem);
    }

    return res;
}

// 使用备忘录存储重复子节点
int CoinChangeHelper(std::vector<int>& memo, std::vector<int>& coins, int amount)
{
    if (memo[amount] != std::numeric_limits<int>::max())
        return memo[amount];

    if (amount < 0)
        return -1;

    if (amount == 0)
        return 0;

    for (auto coin : coins) {
        int sub_problem = CoinChange(coins, amount - coin);

        if (sub_problem == -1)
            continue;

        memo[amount] = std::min(memo[amount], 1 + sub_problem);
    }

    return memo[amount];
}

// 备忘录递归求解
int CoinChangeMemo(std::vector<int>& coins, int amount)
{
    // 初始值为 int 最大值
    // https://blog.csdn.net/fengbingchun/article/details/77922558
    std::vector<int> memo(amount + 1, std::numeric_limits<int>::max());

    return CoinChangeHelper(memo, coins, amount);
}

// 使用 dp table 自底向上求解
int CoinChangeDPTable(std::vector<int>& coins, int amount)
{
    // 数组大小为 amount + 1，初始值也为 amount + 1
    // 因为凑成 amount 金额最多可用 amount 个 1 元硬币，所以初始化为 amount + 1 相当于正无穷
    std::vector<int> dp(amount + 1, amount + 1);

    // base case
    dp[0] = 0;

    // 外层 for 循环在遍历所有状态的所有取值
    for (int i = 0; i < dp.size(); i++) {
        // 内层 for 循环在求所有选择的最小值
        for (auto coin : coins) {
            // 子问题无解，跳过
            // i = 0 时，无法找零，即跳过
            if (i - coin < 0)
                continue;

            // 子问题 dp[i - coin] 已经被求解了
            // 因为是自底向上的解法
            // dp[5] = 1 + min{ dp[5 - 1 = 4], dp[5 - 2 = 3], dp[5 - 5 = 0] }
            dp[i] = std::min(dp[i], 1 + dp[i - coin]);
        }
    }

    return (dp[amount] == amount + 1) ? -1 : dp[amount];
}

// 当前状态只与前面 3 个状态有关，但是前面 3 个状态不是连续的，如何压缩空间??
int CoinChangeDPTableCompression(std::vector<int>& coins, int amount)
{
    // 数组大小为 amount + 1，初始值也为 amount + 1
    // 因为凑成 amount 金额最多可用 amount 个 1 元硬币，所以初始化为 amount + 1 相当于正无穷
    std::vector<int> dp(amount + 1, amount + 1);

    // base case
    dp[0] = 0;

    // 外层 for 循环在遍历所有状态的所有取值
    for (int i = 0; i < dp.size(); i++) {
        // 内层 for 循环在求所有选择的最小值
        for (auto coin : coins) {
            // 子问题无解，跳过
            // i = 0 时，无法找零，即跳过
            if (i - coin < 0)
                continue;

            // 子问题 dp[i - coin] 已经被求解了
            // 因为是自底向上的解法
            // dp[5] = 1 + min{ dp[5 - 1 = 4], dp[5 - 2 = 3], dp[5 - 5 = 0] }
            dp[i] = std::min(dp[i], 1 + dp[i - coin]);
        }
    }

    return (dp[amount] == amount + 1) ? -1 : dp[amount];
}

int main_coin()
{
    std::vector<int> coins;
    coins.push_back(1);
    coins.push_back(2);
    coins.push_back(5);

    std::cout << "CoinChange(coins, 11) = " << CoinChange(coins, 11) << std::endl;

    std::cout << "CoinChangeMemo(coins, 11) = " << CoinChangeMemo(coins, 11) << std::endl;

    std::cout << "CoinChangeDPTable(coins, 11) = " << CoinChangeDPTable(coins, 11) << std::endl;

    std::cout << "CoinChangeDPTableCompression(coins, 11) = "
              << "待完成" << std::endl;

    return 0;
}

/////////////////////////////////////////////////////////////// 动态规划学习 - 状态转移表 求左上角到右下角的最短路径 ///////////////////////////////////////////////////////////////
// https://time.geekbang.org/column/article/75702

int kMinDist = std::numeric_limits<int>::max();

// 先利用回溯算法穷举递归树节点，查找重复子问题，填写状态转移表
void MinDistBT(int i, int j, int dist, int w[][4], int n)
{
    // 右下角 n-1, n-1 这个位置计算完后，返回最短路径
    if (i == n && j == n) {
        if (dist < kMinDist)
            kMinDist = dist;
        return;
    }

    if (i < n) {
        // 往下走，更新 i = i + 1, j = j
        MinDistBT(i + 1, j, dist + w[i][j], w, n);
    }

    if (j < n) {
        // 往右走，更新 i = i, j = j + 1
        MinDistBT(i, j + 1, dist + w[i][j], w, n);
    }
}

// 再将状态转移表的填写过程翻译成 迭代递推 + dp table
int MinDistDP(int matrix[][4], int n)
{
    // n x n 的状态转移表
    int states[n][n];
    int sum = 0;

    for (int j = 0; j < n; ++j) {
        // 初始化 states 的第一行数据
        sum += matrix[0][j];
        states[0][j] = sum;
    }

    sum = 0;

    for (int i = 0; i < n; ++i) {
        // 初始化 states 的第一列数据
        sum += matrix[i][0];
        states[i][0] = sum;
    }

    // 通过第一行和第一列递推当前位置最短距离
    for (int i = 1; i < n; ++i) {
        for (int j = 1; j < n; ++j) {
            states[i][j] = matrix[i][j] + std::min(states[i][j - 1], states[i - 1][j]);
        }
    }

    return states[n - 1][n - 1];
}

int main11()
{

    return 0;
}

/////////////////////////////////////////////////////////////// 动态规划学习 - 状态转移方程 求左上角到右下角的最短路径 ///////////////////////////////////////////////////////////////

// 递归 + 备忘录
int MinDist2(std::vector<std::vector<int>>& matrix, int i, int j, std::vector<std::vector<int>>& mem)
{ 
    if (i == 0 && j == 0)
        return matrix[0][0];

    if (mem[i][j] > 0)
        return mem[i][j];
    
    int min_left = std::numeric_limits<int>::max();
    if (j - 1 >= 0)
        min_left = MinDist2(matrix, i, j - 1, mem);

    int min_up = std::numeric_limits<int>::max();
    if (i - 1 >= 0)
        min_up = MinDist2(matrix, i - 1, j, mem);
    
    mem[i][j] = matrix[i][j] + std::min(min_left, min_up);
    
    return mem[i][j];
}

int main()
{
    std::vector<std::vector<int>> matrix = { { 1, 3, 5, 9 }, { 2, 1, 3, 4 }, { 5, 2, 6, 7 }, { 6, 8, 4, 3 } };
    std::vector<std::vector<int>> mem = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

    std::cout << MinDist2(matrix, 3, 3, mem) << std::endl;

    return 0;
}