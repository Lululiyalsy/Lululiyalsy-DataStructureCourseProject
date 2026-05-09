#pragma once
#include "Common.h"
#include <string>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <memory>

class Edge;

// 抽象节点基类（房间/设施）
class AbstractNode {
protected:
    std::string id;                 // 节点唯一标识
    int floor;                      // 所在楼层
    MYPOINT pos;                      // 位置坐标
    int capacity;                   // 最大容量
    int currentLoad;                // 当前人数
    double baseVelocity;            // 基础移动速度
    double congestionFactor;        // 拥堵因子 [0,1]
    double congestionSensitivity;   // 拥堵敏感度 [1,3]

    std::queue<int> waitingQueue;       // 等待队列
    std::set<int> servingPassengers;    // 正在服务的乘客集合
    double serviceRate;                 // 服务速率（人/秒）
    double serviceTimer;                // 服务计时器
    int maxSimultaneousServices;        // 最大同时服务数

    std::vector<std::vector<int>> occupancyGrid; // 占用网格（-1=障碍, 0=空, >0=乘客ID）
    int gridWidth;                // 网格宽度
    int gridHeight;               // 网格高度
    double cellSize;              // 单元格尺寸（米）

public:
    AbstractNode(const std::string& nodeId, int nodeFloor, MYPOINT position, int cap,
        double vel, double sensitivity = 1.0, double sRate = 1.0, int maxServ = 1);
    virtual ~AbstractNode() = default;

    std::string getId() const { return id; }
    int getFloor() const { return floor; }
    MYPOINT getPos() const { return pos; }
    int getCapacity() const { return capacity; }
    int getCurrentLoad() const { return currentLoad; }
    double getBaseVelocity() const { return baseVelocity; }
    double getVelocity() const { return baseVelocity * (1.0 - congestionFactor * 0.8); }
    double getCongestionFactor() const { return congestionFactor; }
    double getCongestionSensitivity() const { return congestionSensitivity; }
    int getGridWidth() const { return gridWidth; }
    int getGridHeight() const { return gridHeight; }
    double getCellSize() const { return cellSize; }

    void setId(const std::string& val) { id = val; }
    void setFloor(int val) { floor = val; }
    void setPos(const MYPOINT& val) { pos = val; }
    void setCapacity(int val) { if (val > 0) capacity = val; }
    void setBaseVelocity(double val) { if (val > 0) baseVelocity = val; }
    void setCongestionSensitivity(double val) { if (val >= 1.0 && val <= 3.0) congestionSensitivity = val; }

    void initializeGrid();
    bool isCellValid(int x, int y) const { return x >= 0 && x < gridWidth && y >= 0 && y < gridHeight; }
    bool isCellOccupied(int x, int y) const;
    bool isCellObstacle(int x, int y) const;
    bool occupyCell(int x, int y, int passengerId);
    bool releaseCell(int x, int y);
    bool moveCell(int from_x, int from_y, int to_x, int to_y, int passengerId);

    bool canJoinQueue() const;
    bool joinQueue(int passengerId);
    int serveNextPassenger();
    void completeService(int passengerId) { servingPassengers.erase(passengerId); }
    bool isBeingServed(int passengerId) const { return servingPassengers.count(passengerId) > 0; }
    int getQueueLength() const { return waitingQueue.size(); }

    virtual void onPassengerArrive();
    virtual void onPassengerLeave();
    void updateCongestionFactor();
    void updateCongestion() { updateCongestionFactor(); }

    double getServiceRate() const { return serviceRate; } // 服务速率（人/秒）

    virtual bool canEnter() const { return currentLoad < capacity; } // 是否可进入
    virtual bool canExit(const AbstractNode* nextNode = nullptr, const Edge* connectingEdge = nullptr) const; // 是否可离开

    virtual double getPassThroughTime() const; // 节点停留时间
    void assignServingPassengers();              // 分配服务乘客
    virtual double getServiceInterval() const;   // 服务间隔时间
    virtual void update(double deltaTime);       // 更新节点状态

    virtual std::string getTypeName() const = 0; // 类型名称
    virtual std::string getTypeCode() const = 0; // 类型编码
    virtual void render() const = 0;             // 渲染输出

    virtual std::map<std::string, std::string> toProperties() const;     // 序列化为属性表
    virtual void fromProperties(const std::map<std::string, std::string>& props); // 从属性表反序列化
};

// 站厅节点
class HallNode : public AbstractNode {
public:
    HallNode(const std::string& id, int floor, MYPOINT pos, int cap, double vel, double sens)
        : AbstractNode(id, floor, pos, cap, vel, sens, 2.0, 1) {}
    std::string getTypeName() const override { return "\u7ad9\u5385"; }
    std::string getTypeCode() const override { return "HALL"; }
    void render() const override;
};

// 安检节点
class SecurityNode : public AbstractNode {
public:
    int scannerCount;           // 扫描仪数量
    double checkTimePerPerson;  // 每人检查时间
    bool hasBannedItem;         // 是否有违禁品
    SecurityNode(const std::string& id, int floor, MYPOINT pos, int cap, double vel, double sens, int scanners, double timePerPerson, bool banned)
        : AbstractNode(id, floor, pos, cap, vel, sens, 5.0 / timePerPerson, scanners),
        scannerCount(scanners), checkTimePerPerson(timePerPerson), hasBannedItem(banned) {}
    double getServiceInterval() const override { return checkTimePerPerson; }
    std::string getTypeName() const override { return "\u5b89\u68c0"; }
    std::string getTypeCode() const override { return "SECURITY"; }
    void render() const override;
    std::map<std::string, std::string> toProperties() const override;
};

// 售票节点
class TicketNode : public AbstractNode {
public:
    int windowCount;            // 窗口数量
    double buyTimePerPerson;    // 每人购票时间
    bool hasAutoMachine;        // 是否有自助机
    TicketNode(const std::string& id, int floor, MYPOINT pos, int cap, double vel, double sens, int windows, double timePerPerson, bool autoMachine)
        : AbstractNode(id, floor, pos, cap, vel, sens, 5.0 / timePerPerson, windows),
        windowCount(windows), buyTimePerPerson(timePerPerson), hasAutoMachine(autoMachine) {}
    double getServiceInterval() const override { return buyTimePerPerson; }
    std::string getTypeName() const override { return "\u552e\u7968"; }
    std::string getTypeCode() const override { return "TICKET"; }
    void render() const override;
    std::map<std::string, std::string> toProperties() const override;
};

// 闸机节点
class GateNode : public AbstractNode {
public:
    int gateCount;              // 闸机数量
    bool isBidirectional;       // 是否双向
    GateNode(const std::string& id, int floor, MYPOINT pos, int cap, double vel, double sens, int gates, bool bidir)
        : AbstractNode(id, floor, pos, cap, vel, sens, 5.0, gates),
        gateCount(gates), isBidirectional(bidir) {}
    double getServiceInterval() const override { return 1.0; }
    std::string getTypeName() const override { return "\u95f8\u673a"; }
    std::string getTypeCode() const override { return "GATE"; }
    void render() const override;
    std::map<std::string, std::string> toProperties() const override;
};

// 出口节点
class ExitNode : public AbstractNode {
public:
    std::string exitName;       // 出口名称
    std::string connectedStreet;// 连接街道
    bool isOneWay;              // 是否单向
    int totalExits;             // 总出口数
    ExitNode(const std::string& id, int floor, MYPOINT pos, int cap, double vel, double sens, const std::string& name, const std::string& street, bool oneWay, int total)
        : AbstractNode(id, floor, pos, cap, vel, sens, 1.0, total),
        exitName(name), connectedStreet(street), isOneWay(oneWay), totalExits(total) {}
    double getServiceInterval() const override { return 1.0; }
    std::string getTypeName() const override { return "\u51fa\u53e3"; }
    std::string getTypeCode() const override { return "EXIT"; }
    void render() const override;
    std::map<std::string, std::string> toProperties() const override;
};

// 站台节点
class PlatformNode : public AbstractNode {
public:
    std::string lineName;       // 线路名称
    int direction;              // 行驶方向
    int waitCap;                // 候车容量
    bool hasScreenDoor;         // 是否有屏蔽门
    double nextTrainIn;         // 下次列车到达倒计时
    bool isTrainArriving;       // 列车是否正在进站
    double doorOpenTimer;       // 车门开启计时器
    static constexpr double DOOR_OPEN_DURATION = 30.0; // 车门开启持续时间

    PlatformNode(const std::string& id, int floor, MYPOINT pos, int cap, double vel, double sens,
        const std::string& line, int dir, int waitCap, bool screenDoor, double trainIn)
        : AbstractNode(id, floor, pos, cap, vel, sens, 0.5, 1),
        lineName(line), direction(dir), waitCap(waitCap), hasScreenDoor(screenDoor),
        nextTrainIn(trainIn), isTrainArriving(false), doorOpenTimer(0.0), justArrivedEvent(false) {}

    void update(double deltaTime) override;
    std::string getTypeName() const override { return "\u7ad9\u53f0"; }
    std::string getTypeCode() const override { return "PLATFORM"; }
    void render() const override;

    //获取刚到站的瞬间事件（仅触发一帧）
    bool hasJustArrived() const { return justArrivedEvent; }
    bool isTrainArrivingNow() const { return isTrainArriving; }         // 列车是否到站
    bool canAcceptTrainPassengers() const { return currentLoad < capacity * 0.9; } // 是否可接纳列车乘客
    std::map<std::string, std::string> toProperties() const override;
    void fromProperties(const std::map<std::string, std::string>& props) override;

private:
    bool justArrivedEvent; // 刚到站的事件标志
    void handleTrainArrival(double deltaTime);
};

// 楼梯节点
class StairNode : public AbstractNode {
public:
    int stepCount;              // 台阶数
    StairNode(const std::string& id, int floor, MYPOINT pos, int cap, double vel, double sens, int steps)
        : AbstractNode(id, floor, pos, cap, vel, sens, 0.8, 2), stepCount(steps) {}
    std::string getTypeName() const override { return "\u697c\u68af"; }
    std::string getTypeCode() const override { return "STAIR"; }
    void render() const override;
    std::map<std::string, std::string> toProperties() const override;
};

// 节点工厂
class NodeFactory {
public:
    static std::unique_ptr<AbstractNode> createNode(const std::string& typeCode, const std::map<std::string, std::string>& props); // 根据类型编码创建节点
};