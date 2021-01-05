#include <iostream>
#include <string>
#include <vector>

//////////////////////////////////////////////////////////////// 回溯算法 - 全排列问题 ////////////////////////////////////////////////////////////////
// 判断 track 是否包含数字 num
bool InTrack(std::vector<int>& track, int num)
{
    for (auto i : track) {
        if (i == num)
            return true;
    }

    return false;
}

// 路径：记录在 track 中
// 选择列表：nums 中不存在于 track 的那些元素
// 结束条件：nums 中的元素全都在 track 中出现
void BackTrack(std::vector<int>& nums, std::vector<int>& track, std::vector<std::vector<int>>& res)
{
    // 触发结束条件，即路劲长度等于数字个数
    if (track.size() == nums.size()) {
        res.push_back(track);
        return;
    }

    for (int i = 0; i < nums.size(); i++) {
        // 排除不合法的选择，即防止一个全排列数字串中出现重复数字
        if (InTrack(track, nums[i]))
            continue;

        // 做选择，即当前数字满足条件
        track.push_back(nums[i]);

        // 进入决策树下一层，进行下一个数字的选择
        BackTrack(nums, track, res);

        // 如果下一层触发了结束条件或者没有合法的数字可以组成全排列，则取消上一层选择的数字
        track.pop_back();
    }
}

void Permute(std::vector<int>& nums, std::vector<std::vector<int>>& res)
{
    // 记录「路径」，也就是合法的全排列数字串
    std::vector<int> track;
    BackTrack(nums, track, res);
}

int main1()
{
    std::vector<int> nums;
    nums.push_back(1);
    nums.push_back(2);
    nums.push_back(3);

    std::vector<std::vector<int>> res;

    Permute(nums, res);

    for (auto track : res) {
        for (auto i : track)
            std::cout << i;

        std::cout << std::endl;
    }

    return 0;
}

//////////////////////////////////////////////////////////////// 回溯算法 - N 皇后问题 ////////////////////////////////////////////////////////////////

// 是否可以在 board[row][col] 放置皇后？
bool IsValid(std::vector<std::string>& board, int row, int col)
{
    int n = board.size();

    // 检查列 col 是否有皇后互相冲突
    // 当前行不用检查，因为每行只能放 1 个皇后
    for (int i = 0; i < n; i++) {
        if (board[i][col] == 'Q')
            return false;
    }
    
    // 检查右上方是否有皇后互相冲突，不需要检查右下方
    // 因为皇后是从第一行往后放置的，后面的行肯定不会有皇后
    for (int i = row - 1, j = col + 1; i >= 0 && j < n; i--, j++) {
        if (board[i][j] == 'Q')
            return false;
    }

    // 检查左上方是否有皇后互相冲突，左下方同样不用检查
    for (int i = row - 1, j = col - 1; i >= 0 && j >= 0; i--, j--) {
        if (board[i][j] == 'Q')
            return false;
    }

    return true;
}

// 路径：board 中小于 row 的那些行都已经成功放置了皇后
// 选择列表：第 row 行的所有列都是放置皇后的选择
// 结束条件：row 超过 board 的最后一行
void BackTrackNQueens(std::vector<std::string>& board, int row, std::vector<std::vector<std::string>>& res)
{
    // 触发结束条件，即最后一行也放好了
    if (row == board.size()) {
        res.push_back(board);
        return;
    }

    int n = board[row].size();

    for (int col = 0; col < n; col++) {
        // 排除不合法选择，即会产生攻击的位置
        if (!IsValid(board, row, col))
            continue;

        // 做选择，当前位置放一个皇后
        board[row][col] = 'Q';

        // 进入下一行决策
        BackTrackNQueens(board, row + 1, res);

        // 下一行没有合法的位置放置皇后，就撤销上一行选择的皇后，换下一个位置
        board[row][col] = '.';
    }
}

// 路径：board 中小于 row 的那些行都已经成功放置了皇后
// 选择列表：第 row 行的所有列都是放置皇后的选择
// 结束条件：row 超过 board 的最后一行
bool BackTrackNQueensOnce(std::vector<std::string>& board, int row, std::vector<std::vector<std::string>>& res)
{
    // 触发结束条件，即最后一行也放好了
    if (row == board.size()) {
        res.push_back(board);
        return true;
    }

    int n = board[row].size();

    for (int col = 0; col < n; col++) {
        // 排除不合法选择，即会产生攻击的位置
        if (!IsValid(board, row, col))
            continue;

        // 做选择，当前位置放一个皇后
        board[row][col] = 'Q';

        // 进入下一行决策
        // 只返回一个答案
        if(BackTrackNQueensOnce(board, row + 1, res))
            return true;

        // 下一行没有合法的位置放置皇后，就撤销上一行选择的皇后，换下一个位置
        board[row][col] = '.';
    }

    return false;
}

void SolveNQueens(int n, std::vector<std::vector<std::string>>& res)
{
    // '.' 表示空，'Q' 表示皇后，初始化 n x n 空棋盘
    std::vector<std::string> board(n, std::string(n, '.'));
    BackTrackNQueens(board, 0, res);
    //BackTrackNQueensOnce(board, 0, res);
}

int main2()
{
    // 每行代表一个放置皇后的棋盘结果
    std::vector<std::vector<std::string>> res;

    SolveNQueens(8, res);

    for (auto track : res) {
        // 打印每个棋盘
        for (auto i : track)
            std::cout << i << std::endl;

        std::cout << std::endl;
    }

    std::cout << "8 皇后问题共有 " << res.size() << " 种解法" << std::endl;

    return 0;
}