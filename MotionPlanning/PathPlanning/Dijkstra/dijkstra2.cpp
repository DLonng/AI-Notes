// A C++ program for Dijkstra's single source shortest path algorithm.
// The program is for adjacency matrix representation of the graph
// https://www.geeksforgeeks.org/c-program-for-dijkstras-shortest-path-algorithm-greedy-algo-7/

#include <stdio.h>
#include <limits.h>

// Number of vertices in the graph
#define V 7

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
// 求出没有求出最短路径的顶点中，到 src 顶点有最短距离的顶点索引
// dist 表示每个顶点距离 src 的距离，sptSet 表示顶点是否求出最短路径或者包含在最短路径树中？
int minDistance(int dist[], bool sptSet[])
{
    // 把最小值初始化为 int 最大值，用来保存每次的最小距离
    int min = INT_MAX;

    // 最小距离顶点的索引
    int min_index = 0;

    // 遍历每个顶点，保存到 src 的最短距离的顶点索引和距离
    for (int v = 0; v < V; v++) {
        // 只有当前遍历的顶点没有求得最短路径并且距离 src 的距离小于上次求得的最小值
        // 说明当前顶点距离 src 的距离是最短的
        // 第一次执行 min = dist[0] = 0, min_index = 0，0 是 src 顶点
        if (sptSet[v] == false && dist[v] <= min) {
            min = dist[v];
            min_index = v;
        }
    }

    return min_index;
}

// A utility function to print the constructed distance array
// 打印每个顶点到 src 顶点的最短距离
void printSolution(int dist[], int n, int src)
{
    printf("Vertex Distance from Source\n");
    for (int i = 0; i < V; i++)
        printf("%c->%c = %d\n", i + 65, src + 65, dist[i]);
        //printf("%d tt %d\n", i, dist[i]);
}

// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
void dijkstra(int graph[V][V], int src)
{
    // The output array. dist[i] will hold the shortest distance from src to i
    // 保存顶点 i 到 src 的最短距离
    int dist[V];

    // sptSet[i] will be true if vertex i is included in shortest path tree or shortest distance from src to i is finalized
    // 已经求得最短路径的顶点设置为 true
    bool sptSet[V];

    // Initialize all distances as INFINITE and stpSet[] as false
    // 初始化所有顶点到 src 的距离都为无穷大，这里设置为 int 最大值
    // 设置每个节点都没有求得最短路径
    for (int i = 0; i < V; i++) {
        dist[i] = INT_MAX;
        sptSet[i] = false;
    }

    // Distance of source vertex from itself is always 0
    // src 与自己的距离为 0
    dist[src] = 0;

    // Find shortest path for all vertices
    // 使用贪心算法求每个节点距离 src 的最短距离，有多少顶点就循环多少次
    for (int count = 0; count < V - 1; count++) {
        // Pick the minimum distance vertex from the set of vertices not
        // yet processed. u is always equal to src in the first iteration.
        // 在当前未求得最短距离的每个顶点中，返回距离 src 最短的顶点索引
        int u = minDistance(dist, sptSet);

        // Mark the picked vertex as processed
        // 把求出最短距离的顶点标记为 true
        sptSet[u] = true;

        // Update dist value of the adjacent vertices of the picked vertex.
        // 更新与顶点 u 连接的顶点到 src 的最短距离
        // 比如第一次更新 C->D = 3, E->D = 4
        for (int v = 0; v < V; v++) {
            // Update dist[v] only if is not in sptSet, there is an edge from
            // u to v, and total weight of path from src to v through u is
            // smaller than current value of dist[v]
            // 顶点没有求出最短距离，且距离不为 0（距离为 0 的是顶点自身和不直接相连的顶点），且距离不为无穷大，且 v -> u -> src 的距离 < v -> src 的距离
            if ((sptSet[v] == false) && (graph[u][v] != 0) /*&& (dist[u] != INT_MAX)*/ && (dist[u] + graph[u][v] < dist[v]))
                dist[v] = dist[u] + graph[u][v]; // 更新 v 到 src 的最短距离
        }
    }

    // print the constructed distance array
    printSolution(dist, V, src);
}

// driver program to test above function
int main()
{
    /* Let us create the example graph discussed above */
    // 创建邻接矩阵表示图
    /*
    //V = 9
    int graph[V][V] = {
        { 0, 4, 0, 0, 0, 0, 0, 8, 0 },
        { 4, 0, 8, 0, 0, 0, 0, 11, 0 },
        { 0, 8, 0, 7, 0, 4, 0, 0, 2 },
        { 0, 0, 7, 0, 9, 14, 0, 0, 0 },
        { 0, 0, 0, 9, 0, 10, 0, 0, 0 },
        { 0, 0, 4, 14, 10, 0, 2, 0, 0 },
        { 0, 0, 0, 0, 0, 2, 0, 1, 6 },
        { 8, 11, 0, 0, 0, 0, 1, 0, 7 },
        { 0, 0, 2, 0, 0, 0, 6, 7, 0 } };
    */
   
    // 2 个顶点不存在边就是 0
    // https://zhuanlan.zhihu.com/p/40338107
    int graph[V][V] = {
        //A  B   C  D  E  F   G 
        { 0, 12, 0, 0, 0, 16, 14 }, // A 0
        { 12, 0, 10, 0, 0, 7, 0 },  // B 1
        { 0, 10, 0, 3, 5, 6, 0 },   // C 2
        { 0, 0, 3, 0, 4, 0, 0 },    // D 3
        { 0, 0, 5, 4, 0, 2, 8 },    // E 4
        { 16, 7, 6, 0, 2, 0, 9 },   // F 5
        { 14, 0, 0, 0, 8, 9, 0 }    // G 6
        };

    // 计算每个顶点距离顶点 D = 3 的最短距离
    dijkstra(graph, 3);

    return 0;
}