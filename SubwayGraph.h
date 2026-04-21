#pragma once
#include "Enums.h"
#include "Edge.h"
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <queue>

class AbstractNode;

// 地铁拓扑图
class SubwayGraph {
private:
    std::vector<std::unique_ptr<AbstractNode>> nodes_; // 节点列表
    mutable std::vector<std::vector<Edge>> adjList_;   // 邻接表
    std::unordered_map<std::string, int> idToIndex_;   // 节点ID到索引映射
    std::vector<std::string> indexToId_;               // 索引到节点ID映射

    // 寻路算法缓冲区（mutable允许const方法内修改）
    mutable std::vector<double> pathDist_;    // 距离数组
    mutable std::vector<int> pathPrev_;      // 前驱数组
    mutable std::vector<bool> pathVisited_;  // 访问标记
    mutable std::vector<bool> estVisited_;   // 拥堵估计访问标记
    mutable std::vector<int> estLevel_;      // 拥堵估计层级
    mutable std::queue<int> estQueue_;       // 拥堵估计队列
    mutable std::vector<std::pair<double, int>> pqContainer_; // 优先队列容器

    mutable std::vector<double> congestionCache_; // 拥堵缓存
    mutable int congestionCacheFrame_ = -1;       // 缓存帧号
    mutable int currentFrame_ = 0;                // 当前帧号

    void ensurePathBuffers() const;  // 确保寻路缓冲区大小
    void resetPathBuffers(double infVal) const; // 重置寻路缓冲区
    void rebuildCongestionCache() const; // 重建拥堵缓存

    std::vector<int> shortestDistancePath(int startIdx, int endIdx) const; // 最短距离路径
    std::vector<int> shortestTimePath(int startIdx, int endIdx) const;     // 最短时间路径
    std::vector<int> multiObjectivePath(int startIdx, int endIdx) const;   // 多目标优化路径
    double estimateFutureCongestion(int startIdx, int endIdx, int lookAheadSteps = 3) const; // 拥堵预测

public:
    SubwayGraph();
    ~SubwayGraph();

    int addNode(std::unique_ptr<AbstractNode> node); // 添加节点，返回索引
    bool removeNode(const std::string& id);           // 移除节点

    AbstractNode* getNode(const std::string& id) const; // 按ID获取节点
    AbstractNode* getNode(int index) const;               // 按索引获取节点
    const std::vector<std::unique_ptr<AbstractNode>>& getAllNodes() const; // 获取全部节点

    bool addEdge(const std::string& fromId, const std::string& toId, const Edge& edge); // 添加边
    bool removeEdge(const std::string& fromId, const std::string& toId);                   // 移除边

    const Edge* getEdge(const std::string& fromId, const std::string& toId) const; // 按ID获取边
    const Edge* getEdge(int fromIdx, int toIdx) const;                               // 按索引获取边
    Edge* getEdgeMutable(int fromIdx, int toIdx) const;                              // 可变边引用
    Edge* getEdgeMutable(const std::string& fromId, const std::string& toId) const;  // 可变边引用

    const std::vector<Edge>& getNeighbors(int index) const;

    int getIndex(const std::string& id) const;
    const std::string& getId(int index) const;
    bool hasNode(const std::string& id) const;

    std::vector<int> findPath(const std::string& startId, const std::string& endId, PathStrategy strategy) const; // 寻路
    bool canTraverseEdge(int fromIdx, int toIdx) const; // 边是否可通行

    bool loadFromCSV(const std::string& filePath);  // 从CSV加载配置
    bool saveToCSV(const std::string& filePath) const; // 保存配置到CSV
    void visualize() const;                           // 可视化输出
    void update(double deltaTime);                    // 更新图状态

    int getNodeCount() const { return static_cast<int>(nodes_.size()); } // 节点数
    int getEdgeCount() const; // 边数
    void updateAllEdgeCongestion() const; // 更新所有边拥堵
};