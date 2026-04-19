#pragma once

// 边（通道）类
class Edge {
private:
    int toIndex;                    // 目标节点索引
    double length;                  // 通道长度
    double width;                   // 通道宽度
    double baseVelocity;            // 基础通行速度
    bool isEscalator;               // 是否为扶梯
    int maxConcurrentOccupancy;     // 最大并发占用人数
    mutable int currentOccupancy;   // 当前占用人数
    mutable double congestionLevel; // 拥堵程度 [0,1]

    void calculateCapacity();       // 根据宽度计算容量

public:
    Edge();

    int getToIndex() const { return toIndex; }
    void setToIndex(int idx) { toIndex = idx; }

    double getLength() const { return length; }
    void setLength(double l) { length = l; }

    double getWidth() const { return width; }
    void setWidth(double w);            // 设置宽度并重算容量

    double getBaseVelocity() const { return baseVelocity; }
    void setBaseVelocity(double v) { baseVelocity = v; }

    bool getIsEscalator() const { return isEscalator; }
    void setIsEscalator(bool e) { isEscalator = e; }

    int getCapacity() const { return maxConcurrentOccupancy; }
    bool tryEnterEdge() const { return currentOccupancy < maxConcurrentOccupancy; } // 尝试进入

    void addOccupant() const;          // 增加占用者
    void removeOccupant() const;       // 减少占用者

    double getCongestionLevel() const { return congestionLevel; }
    void updateCongestion() const;     // 更新拥堵程度

    bool canEnter() const;             // 是否可进入
    double getPassThroughTime() const; // 计算通行时间
};