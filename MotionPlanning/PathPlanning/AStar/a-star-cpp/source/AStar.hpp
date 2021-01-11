/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>
#include <algorithm>

namespace AStar
{
    // 网格地图坐标
    struct Vec2i
    {
        int x, y;

        bool operator == (const Vec2i& coordinates_);
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    // AStar 算法的每个节点
    struct Node
    {
        // 当前节点到起点的距离
        uint G;

        // 当前节点终端的距离
        uint H;

        // 当前节点在地图上的坐标
        Vec2i coordinates;
        
        // 当前节点的父节点，用于找到终点后反向寻找路径
        Node *parent;

        Node(Vec2i coord_, Node *parent_ = nullptr);

        // 得到当前节点的优先级 F = G + H，F 值越小优先级越高
        uint getScore();
    };

    // 节点向量
    using NodeSet = std::vector<Node*>;

    class Generator
    {
        // 删除障碍物
        bool detectCollision(Vec2i coordinates_);
        // 在节点列表中寻找一个节点
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
        // 释放节点
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void setWorldSize(Vec2i worldSize_);
        void setDiagonalMovement(bool enable_);

        // 设置启发式距离函数
        void setHeuristic(HeuristicFunction heuristic_);
        
        // 寻找路径，返回路径坐标向量
        CoordinateList findPath(Vec2i source_, Vec2i target_);

        // 障碍物操作
        void addCollision(Vec2i coordinates_);
        void removeCollision(Vec2i coordinates_);
        void clearCollisions();

        uint SetBaseCost(uint cur_g, int directions_i);
    private:
        // 启发式距离函数指针
        HeuristicFunction heuristic;
        CoordinateList direction;
        // 障碍物列表
        CoordinateList walls;
        Vec2i worldSize;
        uint directions;
    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        // 曼哈顿距离
        static uint manhattan(Vec2i source_, Vec2i target_);
        // 欧式距离
        static uint euclidean(Vec2i source_, Vec2i target_);
        // 对角距离
        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
