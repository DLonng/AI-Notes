#include <iostream>
#include <vector>
#include <limits>


/**
 * @brief 求一个数组中合法位置的最小值及其索引
 * @details 这里的最小值是每个顶点距离源点 src_v 的最短距离
 * 
 * @param min_dist [in] 顶点到源点最短距离数组
 * @param min_get [in] 是否求得最短距离的 bool 数组
 * @return 距离源点最短距离的顶点索引
 * 
 * @author DLonng
 * @date 2021-01-10
 */
int MinDistance(std::vector<int>& min_dist, std::vector<bool>& min_get)
{
    int min = std::numeric_limits<int>::max();
    int min_index = -1;

    for (int v = 0; v < min_dist.size(); v++) {
        // 只计算没有求出最短路径的顶点
        if ((min_get[v] == false) && (min_dist[v] < min)) {
            min = min_dist[v];
            min_index = v;
        }
    }

    return min_index;
}

/**
 * @brief 单源最短路径 Dijkstra 算法
 * @details 贪心算法
 * 
 * @param graph [in] 邻接矩阵表示的图
 * @param src_v [in] 源点
 * @return 最短距离数组 min_dist
 * 
 * @author DLonng
 * @date 2021-01-10
 * 
 * @note 参考的博客
 *      https://zhuanlan.zhihu.com/p/40338107
 *      https://www.geeksforgeeks.org/c-program-for-dijkstras-shortest-path-algorithm-greedy-algo-7/
 */
std::vector<int> Dijkstra(std::vector<std::vector<int>>& graph, int src_v)
{
    // 保存每个顶点到 src_v 顶点的最短距离
    std::vector<int> min_dist;

    // 保存顶点是否已经求出最短距离
    std::vector<bool> min_get;

    int v_size = graph.size();

    // 初始化最小距离为 int 最大值，意思就是无穷大
    // 初始化所有顶点都没有求得最短路径
    for (int v = 0; v < v_size; v++) {
        min_dist.push_back(std::numeric_limits<int>::max());
        min_get.push_back(false);
    }

    // 源点与自身的距离必须要初始化，用于第一次求出顶点 D
    min_dist[src_v] = 0;

    // 贪心算法: 每次都做当前的最优选择，即每次都选择距离源点最短距离的顶点
    // v_size - 1 是因为最后 1 个顶点 A 不用计算
    // 不减 1 也可以，只是会多一次无效计算
    for (int i = 0; i < v_size; i++) {
        // 得到当前距离源点 src_v 最短距离的顶点索引
        // 这里能够直接得到最小顶点的索引是因为 min_dist 存储的是距离源点 src_v 的最短距离
        int min_u = MinDistance(min_dist, min_get);

        // 把求出最短距离的顶点标记为 true，表示已经求出最短路径了
        min_get[min_u] = true;

        // 计算与当前求出的最短距离顶点 min_u 直接相连的顶点到源点 src_v 的最短距离
        // 虽然这里遍历图中的每个顶点，但在循环内部会对顶点进行过滤
        for (int v = 0; v < v_size; v++) {
            // 只计算没有求出最短路径的顶点
            // 只计算与当前最短距离顶点存在边的顶点(邻接矩阵中元素为 0 表示没有边存在)
            // 只计算合法节点中新的距离小于原始距离的节点
            if ((min_get[v] == false) && (graph[min_u][v] != 0) && (graph[min_u][v] + min_dist[min_u] < min_dist[v]))
                min_dist[v] = graph[min_u][v] + min_dist[min_u];
        }
    }

    return min_dist;
}

void ShowMinDist(std::vector<int>& min_dist, int src_v)
{
    for (int v = 0; v < min_dist.size(); v++)
        std::cout << static_cast<char>(v + 65) << "->" << static_cast<char>(src_v + 65) << " = " << min_dist[v] << std::endl;
}

// 编译: g++ dijkstra.cpp -std=c++11
// 运行: ./a.out
int main()
{
    // 实现: https://zhuanlan.zhihu.com/p/40338107

    // 用邻接矩阵存储图
    std::vector<std::vector<int>> graph;

    std::vector<int> A2other = { 0, 12, 0, 0, 0, 16, 14 };
    std::vector<int> B2other = { 12, 0, 10, 0, 0, 7, 0 };
    std::vector<int> C2other = { 0, 10, 0, 3, 5, 6, 0 };
    std::vector<int> D2other = { 0, 0, 3, 0, 4, 0, 0 };
    std::vector<int> E2other = { 0, 0, 5, 4, 0, 2, 8 };
    std::vector<int> F2other = { 16, 7, 6, 0, 2, 0, 9 };
    std::vector<int> G2other = { 14, 0, 0, 0, 8, 9, 0 };

    graph.push_back(A2other);
    graph.push_back(B2other);
    graph.push_back(C2other);
    graph.push_back(D2other);
    graph.push_back(E2other);
    graph.push_back(F2other);
    graph.push_back(G2other);

    std::vector<int> min_dist = Dijkstra(graph, 3);
    
    /*
        求顶点 D = 3 到其他节点的最短路径:
            A->D = 22
            B->D = 13
            C->D = 3
            D->D = 0
            E->D = 4
            F->D = 6
            G->D = 12
    */
    ShowMinDist(min_dist, 3);

    return 0;
}