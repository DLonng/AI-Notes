#include "AStar.hpp"

using namespace std::placeholders;

bool AStar::Vec2i::operator==(const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator+(const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return { left_.x + right_.x, left_.y + right_.y };
}

AStar::Node::Node(Vec2i coordinates_, Node* parent_)
{
    // 初始化节点的父节点
    parent = parent_;
    // 初始化坐标
    coordinates = coordinates_;
    G = H = 0;
}

// 返回总优先级
AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    // 使用 4 个方向
    setDiagonalMovement(false);

    // 使用曼哈顿距离
    setHeuristic(&Heuristic::manhattan);

    // 初始化所有 8 个方向
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 }, // 上下左右方向
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 } // 四个对角方向
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    // false = 曼哈顿距离，只能上下左右 4 个方向移动
    directions = (enable_ ? 8 : 4);
}

// 绑定启发函数
void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

// 把障碍物坐标加入障碍物列表中
void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

// 查找一个障碍物并从列表中删除
void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

// 清空障碍物列表
void AStar::Generator::clearCollisions()
{
    walls.clear();
}

// 使用 AStar 算法寻找从 source -> target 的最短距离，启发式搜索使用曼哈顿距离
AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    Node* current = nullptr;

    // 待遍历的节点
    NodeSet openSet;

    // 已经遍历过的节点
    NodeSet closedSet;

    // 预留大小为 100 个节点
    openSet.reserve(100);
    closedSet.reserve(100);

    // 把起点加入 open 表中，表示从第一个点开始搜索
    openSet.push_back(new Node(source_));

    // openset 不为空，继续 AStar 搜索
    while (!openSet.empty()) {

        // 得到 open 表的第一个节点
        auto current_it = openSet.begin();
        current = *current_it;

        // 遍历 open set 选择优先级最小的节点，放到 current 中
        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;

            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        // 当前选择的是终点则退出 AStar
        if (current->coordinates == target_) {
            break;
        }

        // 把当前节点放入 close 表，表示已经访问过了
        closedSet.push_back(current);

        // 从 open 表中删除已访问的当前节点
        openSet.erase(current_it);

        // 遍历当前节点的 8 个方向
        for (uint i = 0; i < directions; ++i) {
            // 计算每个方向的新坐标
            Vec2i newCoordinates(current->coordinates + direction[i]);

            // 如果新方向的位置有障碍物或者已经访问过了，就选择下一个方向
            if (detectCollision(newCoordinates) || findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            // 网格之间的移动开销可以自定义
            // 2 个网格之间的水平移动距离开销为 10，即上下左右
            // 2 个网格之间的斜向移动距离开销为 14，即四个对角
            // 新方向的节点距离起点的距离等于 当前节点到起点的距离 + 8 领域网格移动距离
            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            // 在 open 表中查找新方向的坐标节点
            Node* successor = findNodeOnList(openSet, newCoordinates);

            // 不在待遍历的列表中就创建 1 个新方向的节点
            if (successor == nullptr) {
                // 新方向节点的父节点是 current
                successor = new Node(newCoordinates, current);
                // 计算到起点的距离，从 current->G 累加网格移动距离
                //successor->G = totalCost;
                successor->G = SetBaseCost(current->G, i);
                // 计算到终点的启发式距离
                successor->H = heuristic(successor->coordinates, target_);
                // 把新的待遍历的节点放入 open set 中
                openSet.push_back(successor);
            } else if (totalCost < successor->G) {
                // 新方向的节点已经在待遍历的列表中，就更新父节点和 G
                successor->parent = current;
                //successor->G = totalCost;
                successor->G = SetBaseCost(current->G, i);
            }
        }
    }

    // 到达终点后，反向输出路径到 path 中
    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    // 释放资源
    releaseNodes(openSet);
    releaseNodes(closedSet);

    // 返回 AStar 路径
    return path;
}

AStar::uint AStar::Generator::SetBaseCost(uint cur_g, int directions_i)
{
    return cur_g + ((directions_i < 4) ? 10 : 14);
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }

    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x || coordinates_.y < 0 || coordinates_.y >= worldSize.y || std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}


AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return { abs(source_.x - target_.x), abs(source_.y - target_.y) };
}

// 曼哈顿距离
AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

// 欧式距离
AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

// 对角距离
AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    // D = 10, D2 = 14
    // D2 - 2 * D = -6
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
