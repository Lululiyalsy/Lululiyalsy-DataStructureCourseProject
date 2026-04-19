#pragma once
#include "Enums.h"
#include <vector>
#include <string>
#include <queue>
#include <utility>

class AbstractNode;
class SubwayGraph;
class Edge;

// 乘客属性
class PassengerAttributes {
public:
    float speed;            // 速度系数 [0.8, 1.2]
    float patience;         // 耐心值 [0, 1]
    float familiarity;      // 熟悉度 [0, 1]
    bool has_luggage;       // 是否携带行李
    std::string purpose;    // 出行目的 (commute/leisure)

    PassengerAttributes() : speed(1.0f), patience(1.0f), familiarity(1.0f),
        has_luggage(false), purpose("commute") {}
};

// 乘客智能体
class Passenger {
public:
    PassengerState state;           // 当前状态
    PassengerAttributes attributes; // 个性化属性
    PathStrategy pathStrategy;      // 路径规划策略

    int id;                         // 唯一标识
    int current_node_id;            // 当前节点索引
    int target_node_id;             // 目标节点索引
    std::vector<int> path;          // 规划路径（节点索引序列）
    int current_path_index;         // 当前路径位置
    bool isFromTrain;               // 是否从列车下车
    bool headingToPlatform;         // 是否前往站台

    double action_timer;            // 当前动作计时器
    double spawn_time;              // 生成时间
    double exit_time;               // 离开时间

    int queue_start_time;           // 加入队列时间
    int queue_position;             // 队列位置

    int current_grid_x;             // 当前网格X坐标
    int current_grid_y;             // 当前网格Y坐标

    int current_edge_from;          // 当前通道起点索引
    int current_edge_to;            // 当前通道终点索引
    double transit_timer;           // 通行计时器

    double lastReplanTime;          // 上次重规划时间
    double replanInterval;          // 重规划间隔
    double lastCongestionReplanTime;// 上次拥堵重规划时间
    static constexpr double congestionReplanCooldown = 30.0; // 拥堵重规划冷却时间

    int collision_timer;            // 碰撞等待计时器
    std::vector<std::pair<int, int>> collision_path_buffer; // 碰撞局部路径缓存
    const SubwayGraph* graphRef;    // 图引用（用于获取节点信息）

    Passenger(int curr_id, int curr_node, int target_node, PassengerAttributes attrs,
        double time, PathStrategy strategy, bool from_train = false,
        const SubwayGraph* graph = nullptr);

    mutable std::vector<std::vector<bool>> bfs_visited;          // BFS访问标记
    mutable std::vector<std::vector<std::pair<int, int>>> bfs_parent; // BFS父节点记录
    mutable int bfs_buffer_width = 0;  // BFS缓冲区宽度
    mutable int bfs_buffer_height = 0; // BFS缓冲区高度

    void ensureBfsBuffers(int gw, int gh) const; // 确保BFS缓冲区大小

    void setPath(const std::vector<int>& calculatedPath); // 设置规划路径
    std::pair<int, int> getDirectionToTarget(const AbstractNode* node, int target_x, int target_y) const; // 计算移动方向
    std::vector<std::pair<int, int>> findLocalPath(const AbstractNode* node, int target_x, int target_y) const; // BFS局部寻路
    std::string findNearestExit(const SubwayGraph& graph) const; // 查找最近出口

    bool update(double dt, int current_node_load, int current_node_capacity, // 更新乘客状态
        double node_service_time, AbstractNode* current_node,
        const SubwayGraph& graph, int& replansThisFrame, int maxReplansPerFrame = 30);

    void advancePath(); // 推进路径到下一节点
    bool needsReplanning(AbstractNode* currentNode, const SubwayGraph& graph); // 是否需要重规划
    void replanPath(const std::string& currentId, const std::string& targetId, const SubwayGraph& graph); // 执行重规划
    bool isPathCongested(const SubwayGraph& graph) const; // 路径是否拥堵

    std::string get_state_string() const; // 获取状态中文名
    double get_travel_time() const { return exit_time - spawn_time; } // 计算通行时间
    AbstractNode* getNode(int index) const; // 通过图引用获取节点
};