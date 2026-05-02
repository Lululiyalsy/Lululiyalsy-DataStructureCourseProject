#include <iostream> //这是一个整合版的程序
#include <memory>
#include <map>
#include <set>
#include <vector>
#include <queue>
#include <unordered_map>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <random>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <ctime>
#define NOMINMAX
#include <graphics.h>
#include <conio.h>
#include <windows.h>

struct MyPoint
{
    int x;
    int y;
};
using MYPOINT = MyPoint;

// Forward declarations
class AbstractNode;
class Edge;
class Passenger;

// 路径规划策略枚举
enum class PathStrategy
{
    SHORTEST_DISTANCE,
    SHORTEST_TIME,
    MULTI_OBJECTIVE_OPTIMIZATION
};

int safeStoi(const std::string &str, int defaultVal = 0)
{
    if (str.empty() || str == " " || str == "\"\"")
        return defaultVal;
    try
    {
        std::string cleaned = str;
        if (!cleaned.empty() && cleaned.front() == '"')
            cleaned = cleaned.substr(1);
        if (!cleaned.empty() && cleaned.back() == '"')
            cleaned.pop_back();
        if (cleaned.empty())
            return defaultVal;
        return std::stoi(cleaned);
    }
    catch (...)
    {
        return defaultVal;
    }
}

double safeStod(const std::string &str, double defaultVal = 0.0)
{
    if (str.empty() || str == " " || str == "\"\"")
        return defaultVal;
    try
    {
        std::string cleaned = str;
        if (!cleaned.empty() && cleaned.front() == '"')
            cleaned = cleaned.substr(1);
        if (!cleaned.empty() && cleaned.back() == '"')
            cleaned.pop_back();
        if (cleaned.empty())
            return defaultVal;
        return std::stod(cleaned);
    }
    catch (...)
    {
        return defaultVal;
    }
}

class Edge
{
private:
    int toIndex;
    double length;
    double width;
    double baseVelocity;
    bool isEscalator;
    int maxConcurrentOccupancy;
    mutable int currentOccupancy;
    mutable double congestionLevel;

    void calculateCapacity()
    {
        maxConcurrentOccupancy = static_cast<int>(std::ceil(width / 0.5));
        if (maxConcurrentOccupancy < 1)
            maxConcurrentOccupancy = 1;
    }

public:
    Edge() : toIndex(-1), length(10.0), width(2.0), baseVelocity(1.0), isEscalator(false),
             maxConcurrentOccupancy(4), currentOccupancy(0), congestionLevel(0.0)
    {
        calculateCapacity();
    }

    int getToIndex() const { return toIndex; }
    void setToIndex(int idx) { toIndex = idx; }

    double getLength() const { return length; }
    void setLength(double l) { length = l; }

    double getWidth() const { return width; }
    void setWidth(double w)
    {
        width = w;
        calculateCapacity();
    }

    double getBaseVelocity() const { return baseVelocity; }
    void setBaseVelocity(double v) { baseVelocity = v; }

    bool getIsEscalator() const { return isEscalator; }
    void setIsEscalator(bool e) { isEscalator = e; }

    int getCapacity() const { return maxConcurrentOccupancy; }

    bool tryEnterEdge() const { return currentOccupancy < maxConcurrentOccupancy; }

    void addOccupant() const
    {
        currentOccupancy++;
        updateCongestion();
    }
    void removeOccupant() const
    {
        if (currentOccupancy > 0)
            currentOccupancy--;
        updateCongestion();
    }

    double getCongestionLevel() const { return congestionLevel; }

    void updateCongestion() const
    {
        if (maxConcurrentOccupancy > 0)
        {
            congestionLevel = static_cast<double>(currentOccupancy) / maxConcurrentOccupancy;
            if (congestionLevel > 1.0)
                congestionLevel = 1.0;
        }
    }

    bool canEnter() const { return tryEnterEdge(); }

    double getPassThroughTime() const
    {
        double effectiveVelocity = baseVelocity * (isEscalator ? 2.0 : 1.0);
        double congestionPenalty = 1.0 - (congestionLevel * 0.5);
        effectiveVelocity *= congestionPenalty;
        return (effectiveVelocity > 0.001) ? (length / effectiveVelocity) : 999.0;
    }
};

// Abstract Node base class
class AbstractNode
{
protected:
    std::string id;
    int floor;
    MYPOINT pos;
    int capacity;
    int currentLoad;
    double baseVelocity;
    double congestionFactor;
    double congestionSensitivity;

    // 队列相关属性
    std::queue<int> waitingQueue;    // 排队队列
    std::set<int> servingPassengers; // 正在服务的乘客
    double serviceRate;              // 服务速率（人/秒）
    double serviceTimer;             // 服务计时器
    int maxSimultaneousServices;     // 最大同时服务能力

    // 碰撞策略新增：网格化空间表示
    std::vector<std::vector<int>> occupancyGrid; // 网格占用状态：-1=障碍物, 0=空闲, >0=乘客ID
    int gridWidth;
    int gridHeight;
    double cellSize; // 单元格物理尺寸 (米)

public:
    AbstractNode(const std::string &nodeId, int nodeFloor, MYPOINT position, int cap,
                 double vel, double sensitivity = 1.0, double sRate = 1.0, int maxServ = 1)
        : id(nodeId), floor(nodeFloor), pos(position), capacity(cap), currentLoad(0),
          baseVelocity(vel), congestionFactor(0.0), congestionSensitivity(sensitivity),
          serviceRate(sRate), serviceTimer(0.0), maxSimultaneousServices(maxServ),
          gridWidth(20), gridHeight(20), cellSize(0.5)
    { // 初始化网格大小和单元格尺寸
        initializeGrid();
    }

    virtual ~AbstractNode() = default;

    // Getters
    std::string getId() const { return id; }
    int getFloor() const { return floor; }
    MYPOINT getPos() const { return pos; }
    int getCapacity() const { return capacity; }
    int getCurrentLoad() const { return currentLoad; }
    double getBaseVelocity() const { return baseVelocity; }
    double getVelocity() const { return baseVelocity * (1.0 - congestionFactor * 0.8); }
    double getCongestionFactor() const { return congestionFactor; }
    double getCongestionSensitivity() const { return congestionSensitivity; }

    // Grid相关getter方法
    int getGridWidth() const { return gridWidth; }
    int getGridHeight() const { return gridHeight; }
    double getCellSize() const { return cellSize; }

    // Setters
    void setId(const std::string &val) { id = val; }
    void setFloor(int val) { floor = val; }
    void setPos(const MYPOINT &val) { pos = val; }
    void setCapacity(int val)
    {
        if (val > 0)
            capacity = val;
    }
    void setBaseVelocity(double val)
    {
        if (val > 0)
            baseVelocity = val;
    }
    void setCongestionSensitivity(double val)
    {
        if (val >= 1.0 && val <= 3.0)
            congestionSensitivity = val;
    }

    // 碰撞策略新增：初始化网格
    void initializeGrid()
    {
        occupancyGrid.assign(gridWidth, std::vector<int>(gridHeight, 0));
        // 假设四周是障碍物
        for (int i = 0; i < gridWidth; ++i)
        {
            occupancyGrid[i][0] = -1;
            occupancyGrid[i][gridHeight - 1] = -1;
        }
        for (int j = 0; j < gridHeight; ++j)
        {
            occupancyGrid[0][j] = -1;
            occupancyGrid[gridWidth - 1][j] = -1;
        }
    }

    // 碰撞策略新增：网格操作
    bool isCellValid(int x, int y) const
    {
        return x >= 0 && x < gridWidth && y >= 0 && y < gridHeight;
    }

    bool isCellOccupied(int x, int y) const
    {
        if (!isCellValid(x, y))
            return true; // 边界视为被占用
        return occupancyGrid[x][y] != 0;
    }

    bool isCellObstacle(int x, int y) const
    {
        if (!isCellValid(x, y))
            return true; // 边界视为障碍
        return occupancyGrid[x][y] == -1;
    }

    bool occupyCell(int x, int y, int passengerId)
    {
        if (!isCellValid(x, y) || isCellOccupied(x, y))
            return false;
        occupancyGrid[x][y] = passengerId;
        onPassengerArrive();
        return true;
    }

    bool releaseCell(int x, int y)
    {
        if (!isCellValid(x, y) || occupancyGrid[x][y] <= 0)
            return false;
        occupancyGrid[x][y] = 0;
        onPassengerLeave();
        return true;
    }

    bool moveCell(int from_x, int from_y, int to_x, int to_y, int passengerId)
    {
        if (!isCellValid(to_x, to_y) || isCellOccupied(to_x, to_y))
            return false;
        if (!isCellValid(from_x, from_y) || occupancyGrid[from_x][from_y] != passengerId)
            return false;
        occupancyGrid[from_x][from_y] = 0;
        occupancyGrid[to_x][to_y] = passengerId;
        return true;
    }

    // 排队相关方法
    bool canJoinQueue() const
    {
        // 只用队列长度限制，绝对不能用拥堵因子锁死入口
        // 否则任何 >= 0.9 的拥堵都会导致队列永久关闭，形成死循环
        int maxQueueSize = static_cast<int>(capacity * 0.8);
        if (maxQueueSize < 1)
            maxQueueSize = 1;
        return waitingQueue.size() < maxQueueSize;
    }

    bool joinQueue(int passengerId)
    {
        if (canJoinQueue())
        {
            waitingQueue.push(passengerId);
            return true;
        }
        return false;
    }

    int serveNextPassenger()
    {
        if (!waitingQueue.empty() && servingPassengers.size() < maxSimultaneousServices)
        {
            int passengerId = waitingQueue.front();
            waitingQueue.pop();
            servingPassengers.insert(passengerId);
            return passengerId;
        }
        return -1; // 没有可服务的乘客
    }

    void completeService(int passengerId)
    {
        servingPassengers.erase(passengerId);
    }

    bool isBeingServed(int passengerId) const
    {
        return servingPassengers.count(passengerId) > 0;
    }

    int getQueueLength() const
    {
        return waitingQueue.size();
    }

    // Passenger event handling
    virtual void onPassengerArrive()
    {
        if (currentLoad < capacity)
        {
            ++currentLoad;
            updateCongestionFactor();
        }
    }

    virtual void onPassengerLeave()
    {
        if (currentLoad > 0)
        {
            --currentLoad;
            updateCongestionFactor();
        }
    }

    // Congestion calculation
    void updateCongestionFactor()
    {
        double ratio = capacity > 0 ? static_cast<double>(currentLoad) / capacity : 0.0;
        double temp = ratio * congestionSensitivity;
        congestionFactor = (temp < 1.0) ? temp : 1.0;
    }

    // Access checks
    virtual bool canEnter() const { return currentLoad < capacity; }
    virtual bool canExit(const AbstractNode *nextNode = nullptr, const Edge *connectingEdge = nullptr) const
    {
        if (nextNode && !nextNode->canEnter())
        {
            return false;
        }
        if (connectingEdge && !connectingEdge->canEnter())
        {
            return false;
        }
        return servingPassengers.size() < maxSimultaneousServices * 0.9; // 预留少量服务位
    }

    // Pass through time calculation (for Node停留时间)
    virtual double getPassThroughTime() const
    {
        double effectiveVel = getVelocity();
        if (effectiveVel > 0.001)
        {
            return (1.0 / effectiveVel) * (1.0 + congestionFactor);
        }
        return 999.0;
    }

    // AbstractNode新增方法：处理乘客从等待队列进入服务
    void assignServingPassengers()
    {
        while (waitingQueue.size() > 0 && servingPassengers.size() < maxSimultaneousServices)
        {
            int pid = serveNextPassenger();
            if (pid == -1)
                break;
        }
    }

    virtual double getServiceInterval() const
    {
        return 1.0 / serviceRate;
    }

    virtual void update(double deltaTime)
    {
        assignServingPassengers();
        updateCongestionFactor();
    }

    virtual std::string getTypeName() const = 0;
    virtual std::string getTypeCode() const = 0;
    virtual void render() const = 0;

    // Serialization
    virtual std::map<std::string, std::string> toProperties() const
    {
        std::map<std::string, std::string> props;
        props["id"] = id;
        props["type"] = getTypeCode();
        props["floor"] = std::to_string(floor);
        props["x"] = std::to_string(pos.x);
        props["y"] = std::to_string(pos.y);
        props["capacity"] = std::to_string(capacity);
        props["baseVelocity"] = std::to_string(baseVelocity);
        props["congestionFactor"] = std::to_string(congestionFactor);
        props["congestionSensitivity"] = std::to_string(congestionSensitivity);
        props["currentLoad"] = std::to_string(currentLoad);
        return props;
    }

    virtual void fromProperties(const std::map<std::string, std::string> &props)
    {
        auto get = [&](const std::string &key) -> std::string
        {
            auto it = props.find(key);
            return it != props.end() ? it->second : "";
        };
        if (!get("id").empty())
            id = get("id");
        if (!get("floor").empty())
            setFloor(safeStoi(get("floor")));
        if (!get("x").empty() && !get("y").empty())
        {
            pos = {safeStoi(get("x")), safeStoi(get("y"))};
        }
        if (!get("capacity").empty())
            setCapacity(safeStoi(get("capacity")));
        if (!get("baseVelocity").empty())
            setBaseVelocity(safeStod(get("baseVelocity")));
        if (!get("congestionFactor").empty())
            congestionFactor = safeStod(get("congestionFactor"));
        if (!get("congestionSensitivity").empty())
            setCongestionSensitivity(safeStod(get("congestionSensitivity")));
        if (!get("currentLoad").empty())
            currentLoad = safeStoi(get("currentLoad"));
    }
};

// Specialized node types (CorridorNode and EscalatorNode REMOVED)

class HallNode : public AbstractNode
{
public:
    HallNode(const std::string &id, int floor, MYPOINT pos, int cap, double vel, double sens)
        : AbstractNode(id, floor, pos, cap, vel, sens, 2.0, 1) {}

    std::string getTypeName() const override { return "站厅"; }
    std::string getTypeCode() const override { return "HALL"; }
    void render() const override
    {
        std::cout << "站厅: " << id << ", 人数: " << currentLoad << ", 拥堵: " << congestionFactor
                  << ", 队列: " << waitingQueue.size() << std::endl;
    }
};

class SecurityNode : public AbstractNode
{
public:
    int scannerCount;
    double checkTimePerPerson;
    bool hasBannedItem;
    SecurityNode(const std::string &id, int floor, MYPOINT pos, int cap, double vel, double sens, int scanners, double timePerPerson, bool banned)
        : AbstractNode(id, floor, pos, cap, vel, sens, 5.0 / timePerPerson, scanners),
          scannerCount(scanners), checkTimePerPerson(timePerPerson), hasBannedItem(banned) {}

    double getServiceInterval() const override { return checkTimePerPerson; }

    std::string getTypeName() const override { return "安检"; }
    std::string getTypeCode() const override { return "SECURITY"; }
    void render() const override
    {
        std::cout << "安检: " << id << ", 人数: " << currentLoad << ", 拥堵: " << congestionFactor
                  << ", 队列: " << waitingQueue.size() << std::endl;
    }
    std::map<std::string, std::string> toProperties() const override
    {
        auto props = AbstractNode::toProperties();
        props["scannerCount"] = std::to_string(scannerCount);
        props["checkTimePerPerson"] = std::to_string(checkTimePerPerson);
        props["hasBannedItem"] = hasBannedItem ? "1" : "0";
        return props;
    }
};

class TicketNode : public AbstractNode
{
public:
    int windowCount;
    double buyTimePerPerson;
    bool hasAutoMachine;
    TicketNode(const std::string &id, int floor, MYPOINT pos, int cap, double vel, double sens, int windows, double timePerPerson, bool autoMachine)
        : AbstractNode(id, floor, pos, cap, vel, sens, 5.0 / timePerPerson, windows),
          windowCount(windows), buyTimePerPerson(timePerPerson), hasAutoMachine(autoMachine) {}

    double getServiceInterval() const override { return buyTimePerPerson; }

    std::string getTypeName() const override { return "售票"; }
    std::string getTypeCode() const override { return "TICKET"; }
    void render() const override
    {
        std::cout << "售票: " << id << ", 人数: " << currentLoad << ", 拥堵: " << congestionFactor
                  << ", 队列: " << waitingQueue.size() << std::endl;
    }
    std::map<std::string, std::string> toProperties() const override
    {
        auto props = AbstractNode::toProperties();
        props["windowCount"] = std::to_string(windowCount);
        props["buyTimePerPerson"] = std::to_string(buyTimePerPerson);
        props["hasAutoMachine"] = hasAutoMachine ? "1" : "0";
        return props;
    }
};

class GateNode : public AbstractNode
{
public:
    int gateCount;
    bool isBidirectional;
    GateNode(const std::string &id, int floor, MYPOINT pos, int cap, double vel, double sens, int gates, bool bidir)
        : AbstractNode(id, floor, pos, cap, vel, sens, 5.0, gates),
          gateCount(gates), isBidirectional(bidir) {}

    double getServiceInterval() const override { return 1.0; }

    std::string getTypeName() const override { return "闸机"; }
    std::string getTypeCode() const override { return "GATE"; }
    void render() const override
    {
        std::cout << "闸机: " << id << ", 人数: " << currentLoad << ", 拥堵: " << congestionFactor
                  << ", 队列: " << waitingQueue.size() << std::endl;
    }
    std::map<std::string, std::string> toProperties() const override
    {
        auto props = AbstractNode::toProperties();
        props["gateCount"] = std::to_string(gateCount);
        props["isBidirectional"] = isBidirectional ? "1" : "0";
        return props;
    }
};

class ExitNode : public AbstractNode
{
public:
    std::string exitName;
    std::string connectedStreet;
    bool isOneWay;
    int totalExits;
    ExitNode(const std::string &id, int floor, MYPOINT pos, int cap, double vel, double sens, const std::string &name, const std::string &street, bool oneWay, int total)
        : AbstractNode(id, floor, pos, cap, vel, sens, 1.0, total),
          exitName(name), connectedStreet(street), isOneWay(oneWay), totalExits(total) {}

    double getServiceInterval() const override { return 1.0; }

    std::string getTypeName() const override { return "出口"; }
    std::string getTypeCode() const override { return "EXIT"; }
    void render() const override
    {
        std::cout << "出口: " << id << ", 人数: " << currentLoad << ", 拥堵: " << congestionFactor
                  << ", 队列: " << waitingQueue.size() << std::endl;
    }
    std::map<std::string, std::string> toProperties() const override
    {
        auto props = AbstractNode::toProperties();
        props["exitName"] = exitName;
        props["connectedStreet"] = connectedStreet;
        props["isOneWay"] = isOneWay ? "1" : "0";
        props["totalExits"] = std::to_string(totalExits);
        return props;
    }
};

class PlatformNode : public AbstractNode
{
public:
    std::string lineName;
    int direction;
    int waitCap;
    bool hasScreenDoor;
    double nextTrainIn;
    bool isTrainArriving;
    double doorOpenTimer;
    static constexpr double DOOR_OPEN_DURATION = 30.0;

    PlatformNode(const std::string &id, int floor, MYPOINT pos, int cap, double vel, double sens, const std::string &line, int dir, int waitCap, bool screenDoor, double trainIn)
        : AbstractNode(id, floor, pos, cap, vel, sens, 0.5, 1), lineName(line), direction(dir), waitCap(waitCap), hasScreenDoor(screenDoor), nextTrainIn(trainIn), isTrainArriving(false), doorOpenTimer(0.0), justArrivedEvent(false) {}

    void update(double deltaTime) override
    {
        handleTrainArrival(deltaTime);
        AbstractNode::update(deltaTime);
    }

    std::string getTypeName() const override { return "站台"; }
    std::string getTypeCode() const override { return "PLATFORM"; }
    void render() const override { std::cout << "站台: " << id << " (" << lineName << "), 人数: " << currentLoad << ", 拥堵: " << congestionFactor << ", 队列: " << waitingQueue.size() << std::endl; }

public:
    bool isTrainArrivingNow() const { return isTrainArriving; }
    bool canAcceptTrainPassengers() const { return currentLoad < capacity * 0.9; }

    // 【新增】获取刚到站的瞬间事件（仅触发一帧）
    bool hasJustArrived() const { return justArrivedEvent; }

    std::map<std::string, std::string> toProperties() const override
    {
        auto props = AbstractNode::toProperties();
        props["lineName"] = lineName;
        props["direction"] = std::to_string(direction);
        props["waitCap"] = std::to_string(waitCap);
        props["hasScreenDoor"] = hasScreenDoor ? "1" : "0";
        props["nextTrainIn"] = std::to_string(nextTrainIn);
        return props;
    }
    void fromProperties(const std::map<std::string, std::string> &props) override
    {
        AbstractNode::fromProperties(props);
        auto get = [&](const std::string &key) -> std::string
        { auto it = props.find(key); return it != props.end() ? it->second : ""; };
        if (!get("lineName").empty())
            lineName = get("lineName");
        if (!get("direction").empty())
            direction = safeStoi(get("direction"));
        if (!get("waitCap").empty())
            waitCap = safeStoi(get("waitCap"));
        if (!get("hasScreenDoor").empty())
            hasScreenDoor = (get("hasScreenDoor") == "1");
        if (!get("nextTrainIn").empty())
            nextTrainIn = safeStod(get("nextTrainIn"));
    }

private:
    bool justArrivedEvent; // 刚到站的事件标志

    void handleTrainArrival(double deltaTime)
    {
        justArrivedEvent = false; // 【关键】每帧默认重置为 false，确保只触发一次
        if (doorOpenTimer > 0)
        {
            doorOpenTimer -= deltaTime;
            isTrainArriving = true;
        }
        else
        {
            nextTrainIn -= deltaTime;
            if (nextTrainIn <= 0)
            {
                isTrainArriving = true;
                doorOpenTimer = DOOR_OPEN_DURATION;
                nextTrainIn = 120.0;
                justArrivedEvent = true; // 【关键】仅在到站这一瞬间，置为 true
            }
            else
            {
                isTrainArriving = false;
            }
        }
    }
};

class StairNode : public AbstractNode
{
public:
    int stepCount;
    // 更新：已移除 int direction;

    // 更新：修改构造函数，移除 dir 参数
    StairNode(const std::string &id, int floor, MYPOINT pos, int cap, double vel, double sens, int steps)
        : AbstractNode(id, floor, pos, cap, vel, sens, 0.8, 2), stepCount(steps)
    {
    }

    std::string getTypeName() const override { return "楼梯"; }
    std::string getTypeCode() const override { return "STAIR"; }

    void render() const override
    {
        std::cout << "楼梯: " << id << ", 人数: " << currentLoad << ", 拥堵: " << congestionFactor
                  << ", 队列: " << waitingQueue.size() << std::endl;
    }

    std::map<std::string, std::string> toProperties() const override
    {
        auto props = AbstractNode::toProperties();
        props["stepCount"] = std::to_string(stepCount);
        // 更新：移除了 toProperties 中的 direction 映射
        return props;
    }
};

// ==========================================
// 更新：修改 NodeFactory 以适配新的 StairNode 构造
// ==========================================
class NodeFactory
{
public:
    static std::unique_ptr<AbstractNode> createNode(const std::string &typeCode, const std::map<std::string, std::string> &props)
    {
        std::string id = props.at("id");
        int floor = safeStoi(props.at("floor"));
        MYPOINT pos = {safeStoi(props.at("x")), safeStoi(props.at("y"))};
        int cap = safeStoi(props.at("capacity"));
        double vel = safeStod(props.at("baseVelocity"));
        double sens = safeStod(props.at("sensitivity"));

        if (typeCode == "HALL")
        {
            return std::make_unique<HallNode>(id, floor, pos, cap, vel, sens);
        }
        if (typeCode == "SECURITY")
        {
            int scanners = safeStoi(props.at("scannerCount"));
            double t = safeStod(props.at("checkTimePerPerson"));
            bool hasBanned = (props.at("hasBannedItem") == "1");
            return std::make_unique<SecurityNode>(id, floor, pos, cap, vel, sens, scanners, t, hasBanned);
        }
        if (typeCode == "TICKET")
        {
            int win = safeStoi(props.at("windowCount"));
            double t = safeStod(props.at("buyTimePerPerson"));
            bool hasAuto = (props.at("hasAutoMachine") == "1");
            return std::make_unique<TicketNode>(id, floor, pos, cap, vel, sens, win, t, hasAuto);
        }
        if (typeCode == "GATE")
        {
            int g = safeStoi(props.at("gateCount"));
            bool bidir = (props.at("isBidirectional") == "1");
            return std::make_unique<GateNode>(id, floor, pos, cap, vel, sens, g, bidir);
        }
        if (typeCode == "EXIT")
        {
            int total = safeStoi(props.at("totalExits"));
            return std::make_unique<ExitNode>(id, floor, pos, cap, vel, sens, props.at("exitName"), props.at("connectedStreet"), props.at("isOneWay") == "1", total);
        }
        if (typeCode == "PLATFORM")
        {
            return std::make_unique<PlatformNode>(id, floor, pos, cap, vel, sens,
                                                  props.at("lineName"), safeStoi(props.at("direction")),
                                                  safeStoi(props.at("waitCap")),
                                                  props.at("hasScreenDoor") == "1",
                                                  safeStod(props.at("nextTrainIn")));
        }
        // 更新：创建 STAIR 时不再传入 direction 属性
        if (typeCode == "STAIR")
        {
            return std::make_unique<StairNode>(id, floor, pos, cap, vel, sens, safeStoi(props.at("stepCount")));
        }
        return nullptr;
    }
};

// --- Subway Graph Class ---
class SubwayGraph
{
private:
    std::vector<std::unique_ptr<AbstractNode>> nodes_;
    mutable std::vector<std::vector<Edge>> adjList_;
    std::unordered_map<std::string, int> idToIndex_;
    std::vector<std::string> indexToId_;

    mutable std::vector<double> pathDist_;
    mutable std::vector<int> pathPrev_;
    mutable std::vector<bool> pathVisited_;
    mutable std::vector<bool> estVisited_;
    mutable std::vector<int> estLevel_;
    mutable std::queue<int> estQueue_;
    mutable std::vector<std::pair<double, int>> pqContainer_;

    mutable std::vector<double> congestionCache_;
    mutable int congestionCacheFrame_ = -1;
    mutable int currentFrame_ = 0;

    void ensurePathBuffers() const
    {
        size_t n = nodes_.size();
        if (pathDist_.size() != n)
        {
            pathDist_.resize(n);
            pathPrev_.resize(n);
            pathVisited_.resize(n);
            estVisited_.resize(n);
            estLevel_.resize(n);
            congestionCache_.resize(n, 0.0);
        }
    }

    void resetPathBuffers(double infVal) const
    {
        ensurePathBuffers();
        size_t n = nodes_.size();
        std::fill(pathDist_.begin(), pathDist_.end(), infVal);
        std::fill(pathPrev_.begin(), pathPrev_.end(), -1);
        std::fill(pathVisited_.begin(), pathVisited_.end(), false);
    }

    void rebuildCongestionCache() const
    {
        size_t n = nodes_.size();
        if (congestionCache_.size() != n)
            congestionCache_.resize(n, 0.0);
        for (size_t i = 0; i < n; ++i)
        {
            if (nodes_[i])
            {
                congestionCache_[i] = nodes_[i]->getCongestionFactor();
            }
        }
        congestionCacheFrame_ = currentFrame_;
    }

    // 修正后的最短距离路径规划
    std::vector<int> shortestDistancePath(int startIdx, int endIdx) const
    {
        if (startIdx < 0 || endIdx < 0 || startIdx >= nodes_.size() || endIdx >= nodes_.size())
        {
            return {};
        }

        const double INF = 1e18;
        resetPathBuffers(INF);
        pathDist_[startIdx] = 0.0;

        pqContainer_.clear();
        pqContainer_.emplace_back(0.0, startIdx);

        while (!pqContainer_.empty())
        {
            std::pop_heap(pqContainer_.begin(), pqContainer_.end(), std::greater<std::pair<double, int>>());
            auto curr = pqContainer_.back();
            double d = curr.first;
            int u = curr.second;
            pqContainer_.pop_back();
            if (pathVisited_[u])
                continue;
            pathVisited_[u] = true;
            if (u == endIdx)
                break;

            if (u >= adjList_.size())
                continue;

            for (const auto &edge : adjList_[u])
            {
                int v = edge.getToIndex();
                if (v < 0 || v >= static_cast<int>(nodes_.size()))
                    continue;

                // 加入基于节点ID的微小扰动，打破羊群效应，让乘客分散
                // 修改4.25
                size_t hash_seed = std::hash<std::string>{}(indexToId_[u]) ^ (std::hash<std::string>{}(indexToId_[v]) << 1);
                double jitter = (static_cast<double>(hash_seed % 100) / 100.0) * 0.2;
                double weight = edge.getLength() + jitter;

                if (pathDist_[u] + weight < pathDist_[v])
                {
                    pathDist_[v] = pathDist_[u] + weight;
                    pathPrev_[v] = u;
                    pqContainer_.emplace_back(pathDist_[v], v);
                    std::push_heap(pqContainer_.begin(), pqContainer_.end(), std::greater<std::pair<double, int>>());
                }
            }
        }

        std::vector<int> path;
        for (int at = endIdx; at != -1; at = pathPrev_[at])
        {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());
        return (path.front() == startIdx) ? path : std::vector<int>();
    }

    std::vector<int> shortestTimePath(int startIdx, int endIdx) const
    {
        if (startIdx < 0 || endIdx < 0 || startIdx >= nodes_.size() || endIdx >= nodes_.size())
        {
            return {};
        }

        const double INF = 1e18;
        resetPathBuffers(INF);
        pathDist_[startIdx] = 0.0;

        pqContainer_.clear();
        pqContainer_.emplace_back(0.0, startIdx);

        while (!pqContainer_.empty())
        {
            std::pop_heap(pqContainer_.begin(), pqContainer_.end(), std::greater<std::pair<double, int>>());
            auto curr = pqContainer_.back();
            double d = curr.first;
            int u = curr.second;
            pqContainer_.pop_back();
            if (pathVisited_[u])
                continue;
            pathVisited_[u] = true;
            if (u == endIdx)
                break;

            if (u >= adjList_.size())
                continue;

            for (const auto &edge : adjList_[u])
            {
                int v = edge.getToIndex();
                if (v < 0 || v >= static_cast<int>(nodes_.size()))
                    continue;
                if (!nodes_[v])
                    continue;

                double edgeTime = edge.getPassThroughTime();
                double edgeCongestionPenalty = edge.getCongestionLevel() * 5.0;

                double nodeTime = nodes_[v] ? nodes_[v]->getPassThroughTime() : 1.0;
                double nodeCongestionPenalty = nodes_[v] ? nodes_[v]->getCongestionFactor() * 3.0 : 0.0;

                // 加入基于节点ID的微小扰动，打破羊群效应
                // 修改4.25
                size_t hash_seed = std::hash<std::string>{}(indexToId_[u]) ^ (std::hash<std::string>{}(indexToId_[v]) << 1);
                double jitter = (static_cast<double>(hash_seed % 100) / 100.0) * 0.5;
                double weight = edgeTime + edgeCongestionPenalty + nodeTime + nodeCongestionPenalty + jitter;

                if (pathDist_[u] + weight < pathDist_[v])
                {
                    pathDist_[v] = pathDist_[u] + weight;
                    pathPrev_[v] = u;
                    pqContainer_.emplace_back(pathDist_[v], v);
                    std::push_heap(pqContainer_.begin(), pqContainer_.end(), std::greater<std::pair<double, int>>());
                }
            }
        }

        std::vector<int> path;
        for (int at = endIdx; at != -1; at = pathPrev_[at])
        {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());
        return (path.front() == startIdx) ? path : std::vector<int>();
    }

    std::vector<int> multiObjectivePath(int startIdx, int endIdx) const
    {
        if (startIdx < 0 || endIdx < 0 || startIdx >= nodes_.size() || endIdx >= nodes_.size())
        {
            return {};
        }

        const double INF = 1e18;
        resetPathBuffers(INF);
        pathDist_[startIdx] = 0.0;

        if (congestionCacheFrame_ != currentFrame_)
        {
            rebuildCongestionCache();
        }

        pqContainer_.clear();
        pqContainer_.emplace_back(0.0, startIdx);

        while (!pqContainer_.empty())
        {
            std::pop_heap(pqContainer_.begin(), pqContainer_.end(), std::greater<std::pair<double, int>>());
            auto curr = pqContainer_.back();
            double d = curr.first;
            int u = curr.second;
            pqContainer_.pop_back();
            if (pathVisited_[u])
                continue;
            pathVisited_[u] = true;
            if (u == endIdx)
                break;

            if (u >= adjList_.size())
                continue;

            for (const auto &edge : adjList_[u])
            {
                int v = edge.getToIndex();
                if (v < 0 || v >= static_cast<int>(nodes_.size()))
                    continue;
                if (v >= nodes_.size() || !nodes_[v])
                    continue;

                double distanceWeight = edge.getLength();

                double timeWeight = edge.getPassThroughTime() +
                                    (nodes_[v] ? nodes_[v]->getPassThroughTime() : 1.0);

                double currentCongestionWeight = congestionCache_[v];

                double futureCongestionEstimate = estimateFutureCongestion(v, endIdx);

                double congestionWeight = 0.7 * currentCongestionWeight + 0.3 * futureCongestionEstimate;

                double transitionPenalty = 0.0;
                if (nodes_[u] && nodes_[v])
                {
                    std::string fromCode = nodes_[u]->getTypeCode();
                    std::string toCode = nodes_[v]->getTypeCode();
                    if ((fromCode == "SECURITY" && toCode == "HALL") ||
                        (fromCode == "TICKET" && toCode == "HALL") ||
                        (fromCode == "GATE" && toCode == "HALL") ||
                        (fromCode == "GATE" && toCode == "SECURITY"))
                    {
                        transitionPenalty = 50.0;
                    }
                }

                // 加入基于节点ID的微小扰动，打破羊群效应
                // 修改4.25
                size_t hash_seed = std::hash<std::string>{}(indexToId_[u]) ^ (std::hash<std::string>{}(indexToId_[v]) << 1);
                double jitter = (static_cast<double>(hash_seed % 100) / 100.0) * 0.5;
                double weight = 0.3 * distanceWeight + 0.4 * timeWeight + 0.3 * congestionWeight + transitionPenalty + jitter;

                if (pathDist_[u] + weight < pathDist_[v])
                {
                    pathDist_[v] = pathDist_[u] + weight;
                    pathPrev_[v] = u;
                    pqContainer_.emplace_back(pathDist_[v], v);
                    std::push_heap(pqContainer_.begin(), pqContainer_.end(), std::greater<std::pair<double, int>>());
                }
            }
        }

        std::vector<int> path;
        for (int at = endIdx; at != -1; at = pathPrev_[at])
        {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());
        return (path.front() == startIdx) ? path : std::vector<int>();
    }

    double estimateFutureCongestion(int startIdx, int endIdx, int lookAheadSteps = 3) const
    {
        if (startIdx < 0 || endIdx < 0 || startIdx >= nodes_.size())
        {
            return 0.0;
        }

        ensurePathBuffers();
        size_t n = nodes_.size();

        double totalCongestion = 0.0;
        int steps = 0;

        std::fill(estVisited_.begin(), estVisited_.end(), false);
        estVisited_[startIdx] = true;

        while (!estQueue_.empty())
            estQueue_.pop();
        estQueue_.push(startIdx);

        std::fill(estLevel_.begin(), estLevel_.end(), -1);
        estLevel_[startIdx] = 0;

        while (!estQueue_.empty() && steps < lookAheadSteps)
        {
            int u = estQueue_.front();
            estQueue_.pop();

            if (estLevel_[u] > lookAheadSteps)
                break;

            if (u != startIdx)
            {
                if (u < n && nodes_[u])
                {
                    totalCongestion += congestionCache_[u];
                    steps++;
                }
            }

            if (u >= adjList_.size())
                continue;

            for (const auto &edge : adjList_[u])
            {
                int v = edge.getToIndex();
                if (v >= 0 && v < n && !estVisited_[v] && estLevel_[v] == -1)
                {
                    estVisited_[v] = true;
                    estLevel_[v] = estLevel_[u] + 1;
                    estQueue_.push(v);
                }
            }
        }

        return steps > 0 ? totalCongestion / steps : 0.0;
    }

public:
    // 在addNode和removeNode方法中维护图的一致性
    int addNode(std::unique_ptr<AbstractNode> node)
    {
        if (!node)
            return -1;
        std::string id = node->getId();
        if (idToIndex_.count(id))
        {
            std::cout << "节点重复: ID " << id << " 已存在" << std::endl;
            return -1;
        }
        int idx = static_cast<int>(nodes_.size());
        idToIndex_[id] = idx;
        indexToId_.push_back(id);
        nodes_.push_back(std::move(node));
        adjList_.emplace_back();

        pathDist_.resize(nodes_.size());
        pathPrev_.resize(nodes_.size());
        pathVisited_.resize(nodes_.size());
        estVisited_.resize(nodes_.size());
        estLevel_.resize(nodes_.size());
        congestionCache_.resize(nodes_.size(), 0.0);

        return idx;
    }

    bool removeNode(const std::string &id)
    {
        int idx = getIndex(id);
        if (idx < 0 || idx >= nodes_.size())
            return false;

        // 从其他节点的邻接表中移除指向此节点的边
        for (auto &edges : adjList_)
        {
            edges.erase(
                std::remove_if(edges.begin(), edges.end(),
                               [idx](const Edge &e)
                               {
                                   return e.getToIndex() == idx;
                               }),
                edges.end());
        }

        // 清空此节点的邻接表
        adjList_[idx].clear();

        int lastIdx = static_cast<int>(nodes_.size()) - 1;
        if (idx != lastIdx)
        {
            std::string movedId = indexToId_[lastIdx];
            idToIndex_[movedId] = idx;
            indexToId_[idx] = movedId;

            nodes_[idx] = std::move(nodes_[lastIdx]);
            adjList_[idx] = std::move(adjList_[lastIdx]);

            for (auto &edges : adjList_)
            {
                for (auto &edge : edges)
                {
                    if (edge.getToIndex() == lastIdx)
                    {
                        edge.setToIndex(idx);
                    }
                }
            }

            std::cout << "警告: 删除节点 " << id << " (索引" << idx << "), 原末尾节点 " << movedId << " 已移动到索引 " << idx
                      << ", 请确保所有乘客引用的 node ID 已更新为字符串ID而非int索引" << std::endl;
        }

        nodes_.pop_back();
        adjList_.pop_back();
        idToIndex_.erase(id);
        indexToId_.pop_back();

        return true;
    }

    AbstractNode *getNode(const std::string &id) const
    {
        int idx = getIndex(id);
        return (idx >= 0) ? nodes_[idx].get() : nullptr;
    }

    AbstractNode *getNode(int index) const
    {
        return (index >= 0 && index < nodes_.size()) ? nodes_[index].get() : nullptr;
    }

    const std::vector<std::unique_ptr<AbstractNode>> &getAllNodes() const
    {
        return nodes_;
    }

    bool addEdge(const std::string &fromId, const std::string &toId, const Edge &edge)
    {
        int fromIdx = getIndex(fromId);
        int toIdx = getIndex(toId);
        if (fromIdx < 0 || toIdx < 0)
            return false;

        Edge newEdge = edge;
        newEdge.setToIndex(toIdx);
        adjList_[fromIdx].push_back(newEdge);
        return true;
    }

    bool removeEdge(const std::string &fromId, const std::string &toId)
    {
        int fromIdx = getIndex(fromId);
        int toIdx = getIndex(toId);
        if (fromIdx < 0 || toIdx < 0)
            return false;

        auto &edges = adjList_[fromIdx];
        edges.erase(
            std::remove_if(edges.begin(), edges.end(),
                           [toIdx](const Edge &e)
                           { return e.getToIndex() == toIdx; }),
            edges.end());
        return true;
    }

    const Edge *getEdge(const std::string &fromId, const std::string &toId) const
    {
        return getEdge(getIndex(fromId), getIndex(toId));
    }

    const Edge *getEdge(int fromIdx, int toIdx) const
    {
        if (fromIdx < 0 || fromIdx >= static_cast<int>(adjList_.size()))
            return nullptr;
        for (const auto &e : adjList_[fromIdx])
        {
            if (e.getToIndex() == toIdx)
                return &e;
        }
        return nullptr;
    }

    Edge *getEdgeMutable(int fromIdx, int toIdx) const
    {
        if (fromIdx < 0 || fromIdx >= static_cast<int>(adjList_.size()))
            return nullptr;
        for (auto &e : adjList_[fromIdx])
        {
            if (e.getToIndex() == toIdx)
                return &e;
        }
        return nullptr;
    }

    Edge *getEdgeMutable(const std::string &fromId, const std::string &toId) const
    {
        return getEdgeMutable(getIndex(fromId), getIndex(toId));
    }

    const std::vector<Edge> &getNeighbors(int index) const
    {
        static const std::vector<Edge> empty;
        return (index >= 0 && index < adjList_.size()) ? adjList_[index] : empty;
    }

    int getIndex(const std::string &id) const
    {
        auto it = idToIndex_.find(id);
        return (it != idToIndex_.end()) ? it->second : -1;
    }

    const std::string &getId(int index) const
    {
        static const std::string empty;
        return (index >= 0 && index < indexToId_.size()) ? indexToId_[index] : empty;
    }

    bool hasNode(const std::string &id) const
    {
        return idToIndex_.count(id) > 0;
    }

    // 在findPath方法中添加索引有效性检查
    std::vector<int> findPath(const std::string &startId, const std::string &endId, PathStrategy strategy) const
    {
        int startIdx = getIndex(startId);
        int endIdx = getIndex(endId);

        // 检查索引有效性
        if (startIdx < 0 || endIdx < 0)
        {
            std::cout << "路径规划错误: 起点或终点不存在 - " << startId << " -> " << endId << std::endl;
            return {};
        }

        if (startIdx >= nodes_.size() || endIdx >= nodes_.size())
        {
            std::cout << "路径规划错误: 索引超出范围" << std::endl;
            return {};
        }

        switch (strategy)
        {
        case PathStrategy::SHORTEST_DISTANCE:
            return shortestDistancePath(startIdx, endIdx);
        case PathStrategy::SHORTEST_TIME:
            return shortestTimePath(startIdx, endIdx);
        case PathStrategy::MULTI_OBJECTIVE_OPTIMIZATION:
            return multiObjectivePath(startIdx, endIdx);
        default:
            return shortestTimePath(startIdx, endIdx);
        }
    }

    bool canTraverseEdge(int fromIdx, int toIdx) const
    {
        const Edge *e = getEdge(fromIdx, toIdx);
        if (!e)
            return false;
        AbstractNode *next = getNode(toIdx);
        return e->canEnter() && (!next || next->canEnter());
    }

    // 更新：根据指定的 Edge、Node 及所有派生类属性，重新设计统一表头
    bool saveToCSV(const std::string &filePath) const
    {
        std::ofstream file(filePath);
        if (!file.is_open())
        {
            std::cout << "无法创建文件: " << filePath << std::endl;
            return false;
        }

        // 写入CSV统长表头
        file << "recordType,type,id,toId,"
             << "floor,x,y,capacity,baseVelocity,congestionSensitivity,serviceRate,maxSimultaneousServices,"
             << "length,width,isEscalator,maxConcurrentOccupancy,"
             << "scannerCount,checkTimePerPerson,hasBannedItem,"
             << "windowCount,buyTimePerPerson,hasAutoMachine,"
             << "gateCount,isBidirectional,"
             << "exitName,connectedStreet,isOneWay,totalExits,"
             << "lineName,direction,waitCap,hasScreenDoor,nextTrainIn,"
             << "stepCount\n";

        // 批量写入节点 (Nodes)
        for (const auto &node : nodes_)
        {
            std::map<std::string, std::string> props = node->toProperties();
            std::string nodeType = props["type"];

            // 写入基础属性（toId留空）
            file << "NODE," << nodeType << "," << props["id"] << "," << ","
                 << props["floor"] << "," << props["x"] << "," << props["y"] << ","
                 << props["capacity"] << "," << props["baseVelocity"] << ","
                 << props["congestionSensitivity"] << ","
                 // 这里导出通过 serviceInterval 转换的 serviceRate 和 capacity（如果工厂需要，可酌情处理）
                 << (1.0 / std::max(0.01, safeStod(props["checkTimePerPerson"], 1.0))) << ","
                 << "1" << ","; // maxSimultaneousServices

            // Edge专属留空
            file << ",,,,";

            // 特定衍生类属性补齐
            if (nodeType == "SECURITY")
            {
                file << props["scannerCount"] << "," << props["checkTimePerPerson"] << "," << props["hasBannedItem"] << ",";
                file << ",,,";   // ticket
                file << ",,";    // gate
                file << ",,,,";  // exit
                file << ",,,,,"; // platform
                file << "\n";    // stair
            }
            else if (nodeType == "TICKET")
            {
                file << ",,,"; // sec
                file << props["windowCount"] << "," << props["buyTimePerPerson"] << "," << props["hasAutoMachine"] << ",";
                file << ",,";    // gate
                file << ",,,,";  // exit
                file << ",,,,,"; // platform
                file << "\n";    // stair
            }
            else if (nodeType == "GATE")
            {
                file << ",,,"; // sec
                file << ",,,"; // ticket
                file << props["gateCount"] << "," << props["isBidirectional"] << ",";
                file << ",,,,";  // exit
                file << ",,,,,"; // platform
                file << "\n";    // stair
            }
            else if (nodeType == "EXIT")
            {
                file << ",,,"; // sec
                file << ",,,"; // ticket
                file << ",,";  // gate
                file << props["exitName"] << "," << props["connectedStreet"] << "," << props["isOneWay"] << "," << props["totalExits"] << ",";
                file << ",,,,,"; // platform
                file << "\n";    // stair
            }
            else if (nodeType == "PLATFORM")
            {
                file << ",,,";  // sec
                file << ",,,";  // ticket
                file << ",,";   // gate
                file << ",,,,"; // exit
                file << props["lineName"] << "," << props["direction"] << "," << props["waitCap"] << "," << props["hasScreenDoor"] << "," << props["nextTrainIn"] << ",";
                file << "\n"; // stair
            }
            else if (nodeType == "STAIR")
            {
                file << ",,,";   // sec
                file << ",,,";   // ticket
                file << ",,";    // gate
                file << ",,,,";  // exit
                file << ",,,,,"; // platform
                file << props["stepCount"] << "\n";
            }
            else
            {
                file << ",,,,,,," << ",,,,,,," << ",,\n"; // 其余填空补齐
            }
        }

        // 批量写入边 (Edges)
        for (size_t u = 0; u < adjList_.size(); ++u)
        {
            for (const auto &edge : adjList_[u])
            {
                int v = edge.getToIndex();
                if (v < 0 || v >= static_cast<int>(nodes_.size()))
                    continue;

                // EDGE，toId=v对应的id
                file << "EDGE,EDGE," << indexToId_[u] << "," << indexToId_[v] << ",";
                // Node属性留空
                file << ",,,,,,,,";

                // Edge属性填入
                file << edge.getLength() << ","
                     << edge.getWidth() << ","
                     << (edge.getIsEscalator() ? "1" : "0") << ","
                     << edge.getCapacity() << ",";

                // 后续节点细分留空
                file << ",,,,,,,,,,,,,,,,,\n";
            }
        }

        return true;
    }

    // 更新：优化表格字段映射读取逻辑
    bool loadFromCSV(const std::string &filePath)
    {
        std::ifstream file(filePath);
        if (!file.is_open())
        {
            std::cout << "无法打开文件: " << filePath << std::endl;
            return false;
        }

        std::string header;
        std::getline(file, header);

        std::string line;
        int lineNumber = 1;
        int successCount = 0;
        int errorCount = 0;

        std::vector<std::map<std::string, std::string>> nodePropsList;
        std::vector<std::string> nodeTypeCodes;
        std::vector<std::string> edgeFromIds;
        std::vector<std::string> edgeToIds;
        std::vector<Edge> edgeList;

        while (std::getline(file, line))
        {
            lineNumber++;
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            if (line.empty())
                continue;

            try
            {
                std::vector<std::string> values;
                std::string field;
                bool inQuotes = false;
                for (char c : line)
                {
                    if (c == '"')
                    {
                        inQuotes = !inQuotes;
                    }
                    else if (c == ',' && !inQuotes)
                    {
                        values.push_back(field);
                        field.clear();
                    }
                    else
                    {
                        field += c;
                    }
                }
                values.push_back(field);

                std::string recordType = values.size() > 0 ? values[0] : "";
                if (recordType == "NODE" && values.size() >= 12)
                {
                    std::map<std::string, std::string> props;
                    props["type"] = values[1];
                    props["id"] = values[2];
                    props["floor"] = values[4];
                    props["x"] = values[5];
                    props["y"] = values[6];
                    props["capacity"] = values[7];
                    props["baseVelocity"] = values[8];
                    props["sensitivity"] = values[9];

                    std::string typeCode = props["type"];

                    if (typeCode == "SECURITY" && values.size() > 18)
                    {
                        props["scannerCount"] = values[16];
                        props["checkTimePerPerson"] = values[17];
                        props["hasBannedItem"] = values[18];
                    }
                    else if (typeCode == "TICKET" && values.size() > 21)
                    {
                        props["windowCount"] = values[19];
                        props["buyTimePerPerson"] = values[20];
                        props["hasAutoMachine"] = values[21];
                    }
                    else if (typeCode == "GATE" && values.size() > 23)
                    {
                        props["gateCount"] = values[22];
                        props["isBidirectional"] = values[23];
                    }
                    else if (typeCode == "EXIT" && values.size() > 27)
                    {
                        props["exitName"] = values[24];
                        props["connectedStreet"] = values[25];
                        props["isOneWay"] = values[26];
                        props["totalExits"] = values[27];
                    }
                    else if (typeCode == "PLATFORM" && values.size() > 32)
                    {
                        props["lineName"] = values[28];
                        props["direction"] = values[29];
                        props["waitCap"] = values[30];
                        props["hasScreenDoor"] = values[31];
                        props["nextTrainIn"] = values[32];
                    }
                    else if (typeCode == "STAIR" && values.size() > 33)
                    {
                        props["stepCount"] = values[33];
                    }

                    nodePropsList.push_back(props);
                    nodeTypeCodes.push_back(typeCode);
                }
                else if (recordType == "EDGE" && values.size() >= 15)
                {
                    Edge e;
                    e.setLength(safeStod(values[12], 10.0));
                    e.setWidth(safeStod(values[13], 2.0));
                    e.setIsEscalator(safeStoi(values[14], 0) == 1);

                    edgeFromIds.push_back(values[2]);
                    edgeToIds.push_back(values[3]); // fromId, toId使用新排版的位置
                    edgeList.push_back(e);
                }

                successCount++;
            }
            catch (...)
            {
                errorCount++;
            }
        }

        // 批量添加节点
        for (size_t i = 0; i < nodePropsList.size(); ++i)
        {
            auto node = NodeFactory::createNode(nodeTypeCodes[i], nodePropsList[i]);
            if (node)
                addNode(std::move(node));
        }

        // 批量添加边
        for (size_t i = 0; i < edgeFromIds.size(); ++i)
        {
            addEdge(edgeFromIds[i], edgeToIds[i], edgeList[i]);
        }

        std::cout << "CSV加载完成 - 成功记录数: " << successCount << " / 错误忽略: " << errorCount << std::endl;
        return true;
    }

    void visualize() const
    {
        std::cout << "\n=== 地铁站拓扑结构 ===" << std::endl;
        for (size_t i = 0; i < nodes_.size(); ++i)
        {
            std::cout << "节点 " << i << " (" << nodes_[i]->getTypeName() << "): "
                      << nodes_[i]->getId() << " - 楼层 " << nodes_[i]->getFloor()
                      << ", 位置(" << nodes_[i]->getPos().x << "," << nodes_[i]->getPos().y << ")"
                      << ", 负载: " << nodes_[i]->getCurrentLoad() << "/" << nodes_[i]->getCapacity()
                      << ", 拥堵: " << nodes_[i]->getCongestionFactor()
                      << ", 队列: " << nodes_[i]->getQueueLength() << std::endl;

            for (const auto &edge : adjList_[i])
            {
                int to = edge.getToIndex();
                if (to >= 0 && to < nodes_.size())
                {
                    std::cout << "  -> 连接到节点 " << to << " (" << nodes_[to]->getTypeName()
                              << ") 长度: " << edge.getLength() << ", 扶梯: " << (edge.getIsEscalator() ? "是" : "否") << std::endl;
                }
            }
        }
        std::cout << "======================" << std::endl;
    }

    void update(double deltaTime)
    {
        ++currentFrame_;
    }
};

enum class PassengerState
{
    SPAWNED,
    ENTERING,
    TICKETING,
    SECURITY_CHECK,
    MOVING_TO_PLATFORM,
    FROM_TRAIN,
    ON_PLATFORM,
    WAITING_TRAIN,
    BOARDING,
    MOVING_TO_EXIT,
    EXITING,
    LEFT,
    IN_QUEUE,
    PATH_FOLLOWING,
    IN_TRANSIT,
    WAITING_EDGE,
    REPATHING,
    COLLIDING
};

class PassengerAttributes
{
public:
    float speed;         // Speed coefficient (0.8 - 1.2)
    float patience;      // Patience (0.0 - 1.0)
    float familiarity;   // Familiarity (0.0 - 1.0)
    bool has_luggage;    // Has luggage
    std::string purpose; // Purpose ("commute" or "leisure")

    PassengerAttributes() : speed(1.0f), patience(1.0f), familiarity(1.0f),
                            has_luggage(false), purpose("commute") {}
};

class Passenger
{
public:
    PassengerState state;           // Current passenger state
    PassengerAttributes attributes; // Personalized attributes
    PathStrategy pathStrategy;      // 路径规划策略

    int id;                 // Unique identifier
    int current_node_id;    // Current node ID
    int target_node_id;     // Target node ID
    std::vector<int> path;  // Planned path (sequence of node IDs)
    int current_path_index; // Current index in the path
    bool isFromTrain;       // 是否从列车下车
    bool headingToPlatform; // 是否前往站台乘车

    double action_timer; // Timer for current action
    double spawn_time;   // Spawn time (simulation start)
    double exit_time;    // Exit time (simulation end)

    // Queue related attributes
    int queue_start_time; // When joined queue
    int queue_position;   // Position in queue

    // 碰撞策略新增：网格位置追踪
    int current_grid_x;
    int current_grid_y;

    // IN_TRANSIT状态属性
    int current_edge_from;
    int current_edge_to;
    double transit_timer;

    // waiting edge timer
    double waitTimer;
    double real_travel_timer; // 真实通行时间计时器（新增）

    // [新增] 重新规划阈值和上次规划时间
    double lastReplanTime;
    double replanInterval;
    double lastCongestionReplanTime;
    static constexpr double congestionReplanCooldown = 30.0;

    // 碰撞策略新增：等待/避让相关
    int collision_timer;                                    // 等待计时器，防止无限等待
    std::vector<std::pair<int, int>> collision_path_buffer; // 缓存的局部路径
    const SubwayGraph *graphRef;                            // 图引用，用于获取节点信息

    Passenger(int curr_id, int curr_node, int target_node, PassengerAttributes attrs,
              double time, PathStrategy strategy, bool from_train = false,
              const SubwayGraph *graph = nullptr)
        : id(curr_id), current_node_id(curr_node), target_node_id(target_node),
          attributes(attrs), pathStrategy(strategy), spawn_time(time),
          action_timer(0.0), state(from_train ? PassengerState::FROM_TRAIN : PassengerState::SPAWNED),
          exit_time(0.0), queue_start_time(0), queue_position(-1),
          current_path_index(0), current_grid_x(-1), current_grid_y(-1),
          current_edge_from(-1), current_edge_to(-1), transit_timer(0.0), waitTimer(0.0), real_travel_timer(0.0),
          collision_timer(0), isFromTrain(from_train), headingToPlatform(false), lastReplanTime(time), replanInterval(90.0),
          lastCongestionReplanTime(time),
          graphRef(graph) {}

    mutable std::vector<std::vector<bool>> bfs_visited;
    mutable std::vector<std::vector<std::pair<int, int>>> bfs_parent;
    mutable int bfs_buffer_width = 0;
    mutable int bfs_buffer_height = 0;

    void ensureBfsBuffers(int gw, int gh) const
    {
        if (bfs_buffer_width != gw || bfs_buffer_height != gh)
        {
            bfs_visited.assign(gw, std::vector<bool>(gh, false));
            bfs_parent.assign(gw, std::vector<std::pair<int, int>>(gh, {-1, -1}));
            bfs_buffer_width = gw;
            bfs_buffer_height = gh;
        }
        else
        {
            for (auto &row : bfs_visited)
                std::fill(row.begin(), row.end(), false);
            for (auto &row : bfs_parent)
                std::fill(row.begin(), row.end(), std::pair<int, int>(-1, -1));
        }
    }
    // Method to initialize path after creation
    void setPath(const std::vector<int> &calculatedPath)
    {
        path = calculatedPath;
        if (!path.empty())
        {
            current_path_index = 0;
            target_node_id = (path.size() > 1) ? path[1] : path[0];
        }
    }

    // 碰撞策略新增：辅助函数 - 计算到目标网格的简单方向
    std::pair<int, int> getDirectionToTarget(const AbstractNode *node, int target_x, int target_y) const
    {
        int dx = target_x - current_grid_x;
        int dy = target_y - current_grid_y;
        int move_x = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
        int move_y = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
        return {move_x, move_y};
    }

    // GUI所需的getter方法
    int getFloor() const
    {
        if (graphRef)
        {
            const auto &node = graphRef->getNode(current_node_id);
            return node ? node->getFloor() : 0;
        }
        return 0;
    }

    MYPOINT getPosition() const
    {
        if (graphRef)
        {
            const auto &node = graphRef->getNode(current_node_id);
            return node ? node->getPos() : MYPOINT{0, 0};
        }
        return MYPOINT{0, 0};
    }

    PassengerState getState() const
    {
        return state;
    }

    int getCurrentEdgeFrom() const
    {
        return current_edge_from;
    }

    int getCurrentEdgeTo() const
    {
        return current_edge_to;
    }

    double getTransitProgress() const
    {
        // 计算在边上的移动进度（0-1）
        // 这里需要根据实际的边长度和移动时间来计算
        // 简化实现，假设transit_timer与进度成正比
        return std::min(1.0, transit_timer / 10.0); // 假设10秒走完一条边
    }

    // 碰撞策略新增：局部寻路（BFS找最近空位）
    std::vector<std::pair<int, int>> findLocalPath(const AbstractNode *node, int target_x, int target_y) const
    {
        if (current_grid_x < 0 || current_grid_y < 0)
            return {};

        int gw = node->getGridWidth();
        int gh = node->getGridHeight();
        if (current_grid_x >= gw || current_grid_y >= gh)
            return {};

        ensureBfsBuffers(gw, gh);

        std::queue<std::pair<int, int>> q;
        q.push({current_grid_x, current_grid_y});
        bfs_visited[current_grid_x][current_grid_y] = true;

        const int dx[] = {0, 0, 1, -1, 1, -1, 1, -1};
        const int dy[] = {1, -1, 0, 0, 1, 1, -1, -1};

        while (!q.empty())
        {
            auto front = q.front();
            int x = front.first;
            int y = front.second;
            q.pop();

            if (x == target_x && y == target_y && !node->isCellOccupied(x, y))
            {
                std::vector<std::pair<int, int>> path;
                std::pair<int, int> current = {x, y};
                while (current.first != -1)
                {
                    path.push_back(current);
                    current = bfs_parent[current.first][current.second];
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (int i = 0; i < 8; ++i)
            {
                int nx = x + dx[i];
                int ny = y + dy[i];

                if (node->isCellValid(nx, ny) && !bfs_visited[nx][ny] && !node->isCellObstacle(nx, ny))
                {
                    bfs_visited[nx][ny] = true;
                    bfs_parent[nx][ny] = {x, y};
                    q.push({nx, ny});
                }
            }
        }

        return {};
    }
    // [修复] 基于有向图寻路，寻找真正可达的最近出口
    // 修改4.25
    std::string findNearestExit(const SubwayGraph &graph) const
    {
        std::string nearestExitId;
        double minDist = 1e18;

        for (const auto &node : graph.getAllNodes())
        {
            if (node->getTypeCode() == "EXIT")
            {
                std::string exitId = node->getId();
                // 调用最短距离策略验证是否可达，并获取路径
                std::vector<int> path = graph.findPath(graph.getId(current_node_id), exitId, PathStrategy::SHORTEST_DISTANCE);

                if (!path.empty())
                {
                    // 累加路径上所有边的物理长度
                    double dist = 0.0;
                    for (size_t i = 0; i < path.size() - 1; ++i)
                    {
                        const Edge *e = graph.getEdge(path[i], path[i + 1]);
                        if (e)
                            dist += e->getLength();
                    }
                    if (dist < minDist)
                    {
                        minDist = dist;
                        nearestExitId = exitId;
                    }
                }
            }
        }
        return nearestExitId;
    }

    bool update(double dt, int current_node_load, int current_node_capacity,
                double node_service_time, AbstractNode *current_node,
                const SubwayGraph &graph, int &replansThisFrame, int maxReplansPerFrame = 30)
    {

        if (state == PassengerState::SPAWNED)
        {
            action_timer += dt;
            AbstractNode *start_node = current_node;
            if (start_node)
            {
                bool assigned = false;
                if (start_node->occupyCell(1, 1, id))
                {
                    current_grid_x = 1;
                    current_grid_y = 1;
                    assigned = true;
                }
                if (!assigned)
                {
                    for (int x = 1; x < start_node->getGridWidth() - 1 && !assigned; ++x)
                    {
                        for (int y = 1; y < start_node->getGridHeight() - 1; ++y)
                        {
                            if (start_node->occupyCell(x, y, id))
                            {
                                current_grid_x = x;
                                current_grid_y = y;
                                assigned = true;
                                break;
                            }
                        }
                    }
                }
                if (assigned)
                {
                    state = PassengerState::PATH_FOLLOWING;
                }
                else if (action_timer > 10.0)
                {
                    state = PassengerState::LEFT;
                    exit_time = spawn_time + real_travel_timer;
                    return false;
                }
            }
            return true;
        }

        if (state == PassengerState::FROM_TRAIN)
        {
            // 直接转为在站台状态
            state = PassengerState::ON_PLATFORM;
        }

        if (state == PassengerState::ON_PLATFORM)
        {
            if (current_node && current_node->getTypeCode() == "PLATFORM")
            {
                std::string exitId = findNearestExit(graph);
                if (!exitId.empty())
                {
                    std::vector<int> newPath = graph.findPath(
                        graph.getId(current_node_id), exitId, pathStrategy);
                    if (!newPath.empty())
                    {
                        setPath(newPath);
                        state = PassengerState::PATH_FOLLOWING;
                    }
                }
            }
        }

        // [修改] 原有逻辑继续执行
        if (state == PassengerState::COLLIDING)
        {
            if (collision_timer > 2.0)
            {
                collision_timer = 0;
                state = PassengerState::PATH_FOLLOWING;
            }
            return true;
        }

        if (state == PassengerState::LEFT)
            return false;

        action_timer += dt;
        collision_timer += dt;
        real_travel_timer += dt; // 新增每帧累加真实时间
        bool is_currently_congested = (current_node_capacity > 0) &&
                                      (current_node_load * 1.0 / current_node_capacity > 0.8);

        // [新增] 检查当前边的拥堵情况
        bool is_edge_congested = false;
        if (current_path_index > 0)
        {
            std::string prevNodeId = graph.getId(path[current_path_index - 1]);
            std::string currNodeId = graph.getId(current_node_id);
            const Edge *edge = graph.getEdge(prevNodeId, currNodeId);
            if (edge && edge->getCongestionLevel() > 0.8)
            {
                is_edge_congested = true;
            }
        }

        // [已重构] 原逻辑已移至下方PATH_FOLLOWING宏观转移中，此块已禁用
        if (false && current_node_id == target_node_id)
        {
            // 到达站台且要乘车，进入候车状态
            if (headingToPlatform && current_node && current_node->getTypeCode() == "PLATFORM")
            {
                state = PassengerState::WAITING_TRAIN;
                return true;
            }

            if (current_node && (current_node->getTypeCode() == "SECURITY" ||
                                 current_node->getTypeCode() == "TICKET" ||
                                 current_node->getTypeCode() == "GATE" ||
                                 current_node->getTypeCode() == "EXIT"))
            {

                if (!current_node->joinQueue(id))
                {
                    if (attributes.patience < 0.3)
                    {
                        if (current_grid_x >= 0 && current_grid_y >= 0)
                        {
                            current_node->releaseCell(current_grid_x, current_grid_y);
                        }
                        advancePath();
                        if (current_path_index >= path.size())
                        {
                            state = PassengerState::LEFT;
                            exit_time = spawn_time + real_travel_timer;
                        }
                        else
                        {
                            target_node_id = path[current_path_index];
                        }
                        return true;
                    }
                    state = PassengerState::IN_QUEUE;
                    action_timer = 0.0;
                    return true;
                }
                else
                {
                    state = PassengerState::IN_QUEUE;
                    action_timer = 0.0;
                    return true;
                }
            }
            else
            {
                // 不是服务节点，尝试进入边前往下一节点
                if (!path.empty() && current_path_index < path.size() - 1)
                {
                    int next_node_id = path[current_path_index + 1];
                    const Edge *nextEdge = graph.getEdge(current_node_id, next_node_id);
                    if (!nextEdge)
                    {
                        state = PassengerState::REPATHING;
                        return true;
                    }

                    if (!nextEdge->tryEnterEdge())
                    {
                        state = PassengerState::WAITING_EDGE;
                        return true;
                    }

                    AbstractNode *nextNode = getNode(next_node_id);
                    if (nextNode && !nextNode->canEnter())
                    {
                        state = PassengerState::WAITING_EDGE;
                        return true;
                    }

                    // 进入边：释放当前节点网格
                    if (current_node && current_grid_x >= 0 && current_grid_y >= 0)
                    {
                        current_node->releaseCell(current_grid_x, current_grid_y);
                    }

                    // 占用边资源
                    Edge *mutableEdge = graph.getEdgeMutable(current_node_id, next_node_id);
                    if (mutableEdge)
                        mutableEdge->addOccupant();
                    current_edge_from = current_node_id;
                    current_edge_to = next_node_id;
                    transit_timer = 0.0;
                    current_grid_x = -1;
                    current_grid_y = -1;
                    state = PassengerState::IN_TRANSIT;
                    return true;
                }
                else
                {
                    if (current_grid_x >= 0 && current_grid_y >= 0)
                    {
                        current_node->releaseCell(current_grid_x, current_grid_y);
                        current_grid_x = -1;
                        current_grid_y = -1;
                    }
                    state = PassengerState::LEFT;
                    exit_time = spawn_time + real_travel_timer;
                    return true;
                }
            }
        }

        // 处理WAITING_TRAIN状态：在站台候车，列车到站时上车离开
        if (state == PassengerState::WAITING_TRAIN)
        {
            if (current_node && current_node->getTypeCode() == "PLATFORM")
            {
                PlatformNode *platform = dynamic_cast<PlatformNode *>(current_node);
                if (platform && platform->isTrainArrivingNow())
                {
                    if (current_grid_x >= 0 && current_grid_y >= 0)
                    {
                        current_node->releaseCell(current_grid_x, current_grid_y);
                    }
                    state = PassengerState::LEFT;
                    exit_time = spawn_time + real_travel_timer;
                }
            }
            return true;
        }

        // 处理IN_TRANSIT状态：在边上移动
        if (state == PassengerState::IN_TRANSIT)
        {
            const Edge *edge = graph.getEdge(current_edge_from, current_edge_to);
            transit_timer += dt;

            if (edge && transit_timer >= edge->getPassThroughTime())
            {
                AbstractNode *nextNode = getNode(current_edge_to);
                if (nextNode && nextNode->canEnter())
                {
                    // 释放边资源
                    Edge *mutableEdge = graph.getEdgeMutable(current_edge_from, current_edge_to);
                    if (mutableEdge)
                        mutableEdge->removeOccupant();

                    // 到达目标节点
                    current_node_id = current_edge_to;
                    current_path_index++;
                    target_node_id = (current_path_index + 1 < static_cast<int>(path.size()))
                                         ? path[current_path_index + 1]
                                         : path[current_path_index];

                    // 在目标节点分配网格
                    bool assigned = false;
                    if (nextNode->occupyCell(1, 1, id))
                    {
                        current_grid_x = 1;
                        current_grid_y = 1;
                        assigned = true;
                    }
                    if (!assigned)
                    {
                        for (int x = 1; x < nextNode->getGridWidth() - 1 && !assigned; ++x)
                        {
                            for (int y = 1; y < nextNode->getGridHeight() - 1; ++y)
                            {
                                if (nextNode->occupyCell(x, y, id))
                                {
                                    current_grid_x = x;
                                    current_grid_y = y;
                                    assigned = true;
                                    break;
                                }
                            }
                        }
                    }

                    current_edge_from = -1;
                    current_edge_to = -1;
                    transit_timer = 0.0;
                    state = PassengerState::PATH_FOLLOWING;
                }
                //    else {
                //         //修改4.25
                //        if (transit_timer > edge->getPassThroughTime() + 30.0) {
                //             Edge* mutableEdge = graph.getEdgeMutable(current_edge_from, current_edge_to);
                //             if (mutableEdge) mutableEdge->removeOccupant();  // 释放边

                //             // 回退到起点节点
                //             current_node_id = current_edge_from;
                //             current_edge_from = -1;
                //             current_edge_to = -1;
                //             transit_timer = 0.0;

                //             // 重新占据起点的网格
                //             AbstractNode* fromNode = getNode(current_node_id);
                //             if (fromNode) fromNode->occupyCell(1, 1, id);
                //             current_grid_x = 1; current_grid_y = 1;

                //             state = PassengerState::PATH_FOLLOWING;  // 回到可重规划的状态
                //         }
                //    }
            }
            return true;
        }

        // 处理WAITING_EDGE状态：等待边/目标节点可用
        // 修改4.25
        if (state == PassengerState::WAITING_EDGE)
        {
            return true;
        }

        if (state == PassengerState::IN_QUEUE)
        {
            if (current_node)
            {
                if (current_node->isBeingServed(id))
                {
                    double service_time = node_service_time;
                    if (attributes.has_luggage && current_node->getTypeCode() == "SECURITY")
                    {
                        service_time *= 1.5;
                    }
                    service_time /= std::max<float>(0.1f, attributes.speed);

                    if (is_currently_congested)
                    {
                        service_time *= 1.5;
                    }

                    if (action_timer >= service_time)
                    {
                        current_node->completeService(id);
                        // 服务完成，尝试进入边前往下一节点
                        if (!path.empty() && current_path_index < path.size() - 1)
                        {
                            int next_node_id = path[current_path_index + 1];
                            const Edge *nextEdge = graph.getEdge(current_node_id, next_node_id);
                            if (nextEdge && nextEdge->tryEnterEdge())
                            {
                                AbstractNode *nextNode = getNode(next_node_id);
                                if (nextNode && nextNode->canEnter())
                                {
                                    // 释放当前节点网格
                                    if (current_grid_x >= 0 && current_grid_y >= 0)
                                    {
                                        current_node->releaseCell(current_grid_x, current_grid_y);
                                    }
                                    // 进入边
                                    Edge *mutableEdge = graph.getEdgeMutable(current_node_id, next_node_id);
                                    if (mutableEdge)
                                        mutableEdge->addOccupant();
                                    current_edge_from = current_node_id;
                                    current_edge_to = next_node_id;
                                    transit_timer = 0.0;
                                    current_grid_x = -1;
                                    current_grid_y = -1;
                                    state = PassengerState::IN_TRANSIT;
                                }
                                else
                                {
                                    state = PassengerState::PATH_FOLLOWING;
                                }
                            }
                            else
                            {
                                state = PassengerState::PATH_FOLLOWING;
                            }
                        }
                        else
                        {
                            // 路径终点，重新规划路径
                            if (current_node->getTypeCode() != "EXIT" && current_node->getTypeCode() != "PLATFORM")
                            {
                                // 不是出口或站台，重新规划路径
                                std::string currentId = graph.getId(current_node_id);
                                std::string newTargetId = findAppropriateTarget(graph);
                                if (!newTargetId.empty())
                                {
                                    std::vector<int> newPath = graph.findPath(
                                        graph.getId(current_node_id), newTargetId, pathStrategy);
                                    if (!newPath.empty())
                                    {
                                        setPath(newPath);
                                        state = PassengerState::PATH_FOLLOWING;
                                    }
                                    else
                                    {
                                        // 无法规划新路径，离开系统
                                        if (current_grid_x >= 0 && current_grid_y >= 0)
                                        {
                                            current_node->releaseCell(current_grid_x, current_grid_y);
                                        }
                                        state = PassengerState::LEFT;
                                        exit_time = spawn_time + real_travel_timer;
                                    }
                                }
                                else
                                {
                                    // 没有合适的目标，离开系统
                                    if (current_grid_x >= 0 && current_grid_y >= 0)
                                    {
                                        current_node->releaseCell(current_grid_x, current_grid_y);
                                    }
                                    state = PassengerState::LEFT;
                                    exit_time = spawn_time + real_travel_timer;
                                }
                            }
                            else
                            {
                                // 到达出口或站台，离开系统
                                if (current_grid_x >= 0 && current_grid_y >= 0)
                                {
                                    current_node->releaseCell(current_grid_x, current_grid_y);
                                }
                                state = PassengerState::LEFT;
                                exit_time = spawn_time + real_travel_timer;
                            }
                        }
                    }
                }
            }
        }

        // 检查是否需要重新规划路径
        if (needsReplanning(current_node, graph) && replansThisFrame < maxReplansPerFrame)
        {
            state = PassengerState::REPATHING;
            std::string currentId = graph.getId(current_node_id);
            std::string targetId = graph.getId(target_node_id);
            replanPath(currentId, targetId, graph);
            state = PassengerState::PATH_FOLLOWING;
            replansThisFrame++;
        }

        // 只要处于PATH_FOLLOWING状态且路径有效，就尝试转移
        // ==========================================
        if (state == PassengerState::PATH_FOLLOWING && !path.empty())
        {

            // 1. 服务节点必须先排队（注意：EXIT不需要排队，走到出口直接离场）
            // 修改4.25
            if (current_node && (current_node->getTypeCode() == "SECURITY" || current_node->getTypeCode() == "TICKET" || current_node->getTypeCode() == "GATE"))
            {
                if (current_node->joinQueue(id))
                {
                    // 只有真正排进去了，才变成排队状态
                    state = PassengerState::IN_QUEUE;
                    action_timer = 0.0;
                }
                else
                {
                    // 排队失败（比如队列满了、节点太堵）
                    if (attributes.patience < 0.3)
                    {
                        // 没耐心：直接放弃当前节点，强制跳到路径下一个节点
                        if (current_grid_x >= 0 && current_grid_y >= 0)
                            current_node->releaseCell(current_grid_x, current_grid_y);
                        advancePath();
                        if (current_path_index >= static_cast<int>(path.size()))
                        {
                            state = PassengerState::LEFT;
                            exit_time = spawn_time + real_travel_timer;
                            return false;
                        }
                    }
                    else
                    {
                        // 有耐心：保持在 PATH_FOLLOWING 状态原地等待
                        // 绝对不能写成 state = PassengerState::IN_QUEUE; 否则会变成幽灵卡死！
                    }
                }
                return true;
            }
            // 修改结束

            // 2. 出口节点直接离场，绝不排队
            if (current_node && current_node->getTypeCode() == "EXIT")
            {
                if (current_grid_x >= 0 && current_grid_y >= 0)
                {
                    current_node->releaseCell(current_grid_x, current_grid_y);
                    current_grid_x = -1;
                    current_grid_y = -1;
                }
                state = PassengerState::LEFT;
                exit_time = spawn_time + real_travel_timer;
                return false;
            }

            // 3. 站台候车
            if (headingToPlatform && current_node && current_node->getTypeCode() == "PLATFORM")
            {
                state = PassengerState::WAITING_TRAIN;
                return true;
            }

            // 3. 【关键修复】随时尝试进入下一条边（打破死循环的唯一出路）
            if (current_path_index < static_cast<int>(path.size()) - 1)
            {
                int next_node_id = path[current_path_index + 1];
                const Edge *nextEdge = graph.getEdge(current_node_id, next_node_id);
                if (!nextEdge)
                {
                    state = PassengerState::REPATHING;
                    return true;
                }
                AbstractNode *nextNode = getNode(next_node_id);
                if (!nextEdge->tryEnterEdge() || (nextNode && !nextNode->canEnter()))
                {
                    // 【根治修改】：不再切换状态为 WAITING_EDGE！
                    // 原地累加等待时间，并将超时重规划逻辑内聚在这里，然后故意不 return，让代码自然穿透到下方的微观移动逻辑！
                    waitTimer += dt;
                    if (waitTimer > 60.0)
                    {
                        if (needsReplanning(current_node, graph))
                        {
                            std::string currentId = graph.getId(current_node_id);
                            std::string targetId = graph.getId(target_node_id);
                            replanPath(currentId, targetId, graph);
                        }
                        waitTimer = 0.0;
                    }
                    // 注意这里绝对不能写 return，必须穿透！
                }
                else
                {
                    if (current_grid_x >= 0 && current_grid_y >= 0)
                        current_node->releaseCell(current_grid_x, current_grid_y);
                    Edge *mutableEdge = graph.getEdgeMutable(current_node_id, next_node_id);
                    if (mutableEdge)
                        mutableEdge->addOccupant();
                    current_edge_from = current_node_id;
                    current_edge_to = next_node_id;
                    transit_timer = 0.0;
                    current_grid_x = -1;
                    current_grid_y = -1;
                    state = PassengerState::IN_TRANSIT;
                    waitTimer = 0.0; // 成功进边，重置等待计时器
                    return true;
                }
            }
            else
            {
                if (current_grid_x >= 0 && current_grid_y >= 0)
                {
                    current_node->releaseCell(current_grid_x, current_grid_y);
                    current_grid_x = -1;
                    current_grid_y = -1;
                }
                state = PassengerState::LEFT;
                exit_time = spawn_time + real_travel_timer;
                return false;
            }
        }

        // 碰撞策略核心：在节点内部移动（降级为等待进边时的防重叠行为）
        // 上方已经允许穿透，所有被边阻断的节点（HALL、STAIR甚至安检门口）都会掉到这里
        // 使用轻量级随机游走，彻底取代原本极其消耗性能且容易死锁的BFS逻辑
        if (state == PassengerState::PATH_FOLLOWING && current_node && current_grid_x >= 0 && current_grid_y >= 0)
        {
            collision_timer += dt; // 累加碰撞等待时间

            // 每隔 0.5 秒尝试走一步（避免每帧计算浪费性能）
            if (collision_timer >= 0.5)
            {
                const int dx[] = {0, 0, 1, -1};
                const int dy[] = {1, -1, 0, 0};

                // 随机打乱四个移动方向
                int dirs[4] = {0, 1, 2, 3};

                // 随机打乱四个移动方向，避免死锁
                static thread_local std::mt19937 local_rng(std::random_device{}());
                std::shuffle(dirs, dirs + 4, local_rng);

                bool moved = false;
                for (int i = 0; i < 4; ++i)
                {
                    int nx = current_grid_x + dx[dirs[i]];
                    int ny = current_grid_y + dy[dirs[i]];

                    // 只要这个方向不是墙、不是人，就走过去
                    if (!current_node->isCellObstacle(nx, ny) && !current_node->isCellOccupied(nx, ny))
                    {
                        current_node->moveCell(current_grid_x, current_grid_y, nx, ny, id);
                        current_grid_x = nx;
                        current_grid_y = ny;
                        moved = true;
                        break; // 走通一步就停下，等下个 0.5 秒
                    }
                }

                if (moved)
                {
                    collision_timer = 0.0; // 成功移动，清零计时器
                }
                // 如果四面被死死包围，moved为false，collision_timer会继续累加，但永远不会触发死循环BFS
            }
        }

        return true;
    }

    // 辅助方法：前进到路径的下一个节点
    void advancePath()
    {
        if (current_path_index < static_cast<int>(path.size()) - 1)
        {
            current_path_index++;
            target_node_id = (current_path_index + 1 < static_cast<int>(path.size()))
                                 ? path[current_path_index + 1]
                                 : path[current_path_index];
        }
    }

    // 辅助方法：寻找合适的目标节点
    std::string findAppropriateTarget(const SubwayGraph &graph)
    {
        // 寻找最近的出口
        std::string exitId = findNearestExit(graph);
        if (!exitId.empty())
        {
            return exitId;
        }

        // 寻找最近的站台
        std::string platformId = findNearestPlatform(graph);
        if (!platformId.empty())
        {
            return platformId;
        }

        // 寻找最近的楼梯
        std::string stairId = findNearestStair(graph);
        if (!stairId.empty())
        {
            return stairId;
        }

        return "";
    }

    // 辅助方法：寻找最近的出口
    std::string findNearestExit(const SubwayGraph &graph)
    {
        double minDistance = std::numeric_limits<double>::max();
        std::string nearestExit = "";

        for (const auto &node : graph.getAllNodes())
        {
            if (node->getTypeCode() == "EXIT")
            {
                MYPOINT nodePos = node->getPos();
                MYPOINT currentPos = getPosition();
                double distance = std::sqrt(std::pow(nodePos.x - currentPos.x, 2) + std::pow(nodePos.y - currentPos.y, 2));
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestExit = node->getId();
                }
            }
        }

        return nearestExit;
    }

    // 辅助方法：寻找最近的站台
    std::string findNearestPlatform(const SubwayGraph &graph)
    {
        double minDistance = std::numeric_limits<double>::max();
        std::string nearestPlatform = "";

        for (const auto &node : graph.getAllNodes())
        {
            if (node->getTypeCode() == "PLATFORM")
            {
                MYPOINT nodePos = node->getPos();
                MYPOINT currentPos = getPosition();
                double distance = std::sqrt(std::pow(nodePos.x - currentPos.x, 2) + std::pow(nodePos.y - currentPos.y, 2));
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestPlatform = node->getId();
                }
            }
        }

        return nearestPlatform;
    }

    // 辅助方法：寻找最近的楼梯
    std::string findNearestStair(const SubwayGraph &graph)
    {
        double minDistance = std::numeric_limits<double>::max();
        std::string nearestStair = "";

        for (const auto &node : graph.getAllNodes())
        {
            if (node->getTypeCode() == "STAIR")
            {
                MYPOINT nodePos = node->getPos();
                MYPOINT currentPos = getPosition();
                double distance = std::sqrt(std::pow(nodePos.x - currentPos.x, 2) + std::pow(nodePos.y - currentPos.y, 2));
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestStair = node->getId();
                }
            }
        }

        return nearestStair;
    }

    // 检查是否需要重新规划路径
    bool needsReplanning(AbstractNode *currentNode, const SubwayGraph &graph)
    {
        double now = spawn_time + action_timer;
        bool congestionTriggered = false;

        if (currentNode && currentNode->getCongestionFactor() > 0.8)
        {
            congestionTriggered = true;
        }

        if (!congestionTriggered && current_path_index < path.size() - 1)
        {
            for (int i = current_path_index + 1; i < path.size() && i < current_path_index + 3; i++)
            {
                AbstractNode *futureNode = graph.getNode(path[i]);
                if (futureNode && futureNode->getCongestionFactor() > 0.85)
                {
                    congestionTriggered = true;
                    break;
                }
                if (i > 0)
                {
                    const Edge *edge = graph.getEdge(path[i - 1], path[i]);
                    if (edge && edge->getCongestionLevel() > 0.8)
                    {
                        congestionTriggered = true;
                        break;
                    }
                }
            }
        }

        if (congestionTriggered && now - lastCongestionReplanTime > congestionReplanCooldown)
        {
            return true;
        }

        if (now - lastReplanTime > replanInterval)
        {
            return true;
        }

        if (attributes.speed < 0.5 && action_timer > 30.0)
        {
            return true;
        }

        return false;
    }

    // 重新规划路径
    void replanPath(const std::string &currentId, const std::string &targetId, const SubwayGraph &graph)
    {
        PathStrategy currentStrategy = pathStrategy;
        bool wasCongestionTriggered = isPathCongested(graph);

        if (wasCongestionTriggered)
        {
            currentStrategy = PathStrategy::MULTI_OBJECTIVE_OPTIMIZATION;
        }

        std::vector<int> newPath = graph.findPath(currentId, targetId, currentStrategy);
        if (!newPath.empty())
        {
            path = newPath;
            current_path_index = 0;
            target_node_id = path[0];
            double now = spawn_time + action_timer;
            lastReplanTime = now;
            if (wasCongestionTriggered)
            {
                lastCongestionReplanTime = now;
            }
        }
    }
    // [新增] 检查当前路径是否拥堵
    bool isPathCongested(const SubwayGraph &graph) const
    {
        for (int i = current_path_index; i < path.size() - 1; i++)
        {
            AbstractNode *node = graph.getNode(path[i]);
            if (node && node->getCongestionFactor() > 0.7)
            {
                return true;
            }

            // 检查边的拥堵
            if (i > 0)
            {
                const Edge *edge = graph.getEdge(path[i - 1], path[i]);
                if (edge && edge->getCongestionLevel() > 0.7)
                {
                    return true;
                }
            }
        }
        return false;
    }

    std::string get_state_string() const
    {
        switch (state)
        {
        case PassengerState::SPAWNED:
            return "生成";
        case PassengerState::ENTERING:
            return "进站";
        case PassengerState::TICKETING:
            return "购票";
        case PassengerState::SECURITY_CHECK:
            return "安检";
        case PassengerState::MOVING_TO_PLATFORM:
            return "去站台";
        case PassengerState::FROM_TRAIN:
            return "列车下车"; // [新增]
        case PassengerState::ON_PLATFORM:
            return "站台等待"; // [新增]
        case PassengerState::WAITING_TRAIN:
            return "候车";
        case PassengerState::BOARDING:
            return "乘车";
        case PassengerState::MOVING_TO_EXIT:
            return "去出口";
        case PassengerState::EXITING:
            return "出站";
        case PassengerState::LEFT:
            return "离开";
        case PassengerState::IN_QUEUE:
            return "排队";
        case PassengerState::PATH_FOLLOWING:
            return "路径跟随";
        case PassengerState::IN_TRANSIT:
            return "通道中";
        case PassengerState::WAITING_EDGE:
            return "等待通道";
        case PassengerState::REPATHING:
            return "重新规划";
        case PassengerState::COLLIDING:
            return "碰撞等待";
        default:
            return "未知";
        }
    }

    double get_travel_time() const
    {
        return exit_time - spawn_time;
    }

    AbstractNode *getNode(int index) const
    {
        if (graphRef)
        {
            return graphRef->getNode(index);
        }
        return nullptr;
    }
};

class CrowdProfile
{
public:
    std::string name;    // Profile name
    double arrival_rate; // Arrival rate (people/sec)
    float familiarity_min, familiarity_max;
    float patience_min, patience_max;
    float speed_min, speed_max;
    float luggage_prob;  // Luggage probability
    float commute_ratio; // Commute ratio

    CrowdProfile()
        : arrival_rate(0.5),
          familiarity_min(0.5f), familiarity_max(1.0f),
          patience_min(0.5f), patience_max(1.0f),
          speed_min(0.8f), speed_max(1.2f),
          luggage_prob(0.2f), commute_ratio(0.8f) {}
};

class TimeSlot
{
public:
    int start_second; // Start second of day
    int end_second;   // End second of day
    int day_mask;     // Bit mask: 1=Monday, 64=Sunday
    CrowdProfile profile;

    static int time_to_seconds(int hour, int minute)
    {
        return 3600 * hour + minute * 60;
    }
    static int weekday_mask()
    {
        return (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);
    }
    static int weekend_mask()
    {
        return (1 << 5) | (1 << 6);
    }
    static int daily_mask()
    {
        return 0b1111111;
    }
};

class VirtualClock
{
private:
    double total_sim_seconds;
    int start_day_of_week;
    int start_hour_of_day;

public:
    VirtualClock(int start_weekday = 0, int start_hour = 5)
        : total_sim_seconds(start_hour * 3600), start_day_of_week(start_weekday),
          start_hour_of_day(start_hour) {}

    void update(double dt)
    {
        total_sim_seconds += dt;
    }

    int get_current_weekday() const
    {
        int total_days = static_cast<int>(total_sim_seconds / 86400);
        return (start_day_of_week + total_days) % 7;
    }

    int get_seconds_today() const
    {
        return static_cast<int>(total_sim_seconds) % 86400;
    }

    int get_current_hour() const
    {
        return get_seconds_today() / 3600;
    }

    int get_current_minute() const
    {
        return (get_seconds_today() % 3600) / 60;
    }

    std::string get_formatted_time() const
    {
        int weekday = get_current_weekday();
        const char *weekdays[] = {"周一", "周二", "周三", "周四", "周五", "周六", "周日"};
        std::ostringstream oss;
        oss << weekdays[weekday] << " "
            << std::setfill('0') << std::setw(2) << get_current_hour() << ":"
            << std::setfill('0') << std::setw(2) << get_current_minute();
        return oss.str();
    }

    bool is_weekday() const
    {
        int weekday = get_current_weekday();
        return weekday >= 0 && weekday <= 4;
    }

    bool is_weekend() const
    {
        return !is_weekday();
    }

    double get_total_seconds() const
    {
        return total_sim_seconds;
    }
};

struct GenerationStats
{
    int weekday_total = 0;
    int weekend_total = 0;
    int peak_total = 0;
    int offpeak_total = 0;
    std::unordered_map<std::string, int> profile_counts;
};

class PassengerGenerator
{
private:
    std::vector<TimeSlot> schedule;
    VirtualClock &clock;
    mutable std::mt19937 rng; // 修改后可以在const函数中修改
    int total_generated;
    GenerationStats stats;
    SubwayGraph &graphRef; // 引用图
    // [新增] 用于生成从列车下车的乘客
    std::vector<std::string> platformIds;
    int trainPassengerCounter;
    int maxOnlinePassengers;

public:
    PassengerGenerator(VirtualClock &c, SubwayGraph &graph, int maxOnline = 2500)
        : clock(c), graphRef(graph), total_generated(0), trainPassengerCounter(0), maxOnlinePassengers(maxOnline)
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        rng.seed(seed);
        initialize_default_schedule();

        // [新增] 初始化站台ID列表
        updateStationLayout();
    }
    // [新增] 更新车站布局信息
    void updateStationLayout()
    {
        platformIds.clear();
        for (const auto &node : graphRef.getAllNodes())
        {
            if (node->getTypeCode() == "PLATFORM")
            {
                platformIds.push_back(node->getId());
            }
        }
    }
    // [新增] 生成从列车下车的乘客
    std::vector<Passenger> generateTrainPassengers(double dt, int remaining = 9999)
    {
        std::vector<Passenger> train_passengers;
        if (remaining <= 0)
            return train_passengers;

        for (const auto &platformId : platformIds)
        {
            if (remaining <= 0)
                break;
            PlatformNode *platform = dynamic_cast<PlatformNode *>(graphRef.getNode(platformId));

            // 【关键修复1】：从持续30秒的"门是否开着"改为"是否是刚到站这一帧"
            if (platform && platform->hasJustArrived() && platform->canAcceptTrainPassengers())
            {

                // 【关键修复2】：一列车合理下车人数（比如40人），而不是每秒8人持续喷30秒
                int numPassengers = std::min(40, remaining);

                for (int i = 0; i < numPassengers; ++i)
                {
                    trainPassengerCounter++;
                    total_generated++;
                    // 优先选择同楼层的出口
                    int platformFloor = platform->getFloor();
                    std::string exitId = findRandomExit(platformFloor);
                    if (!exitId.empty())
                    {
                        double currentAbsTime = clock.get_total_seconds();
                        Passenger p(trainPassengerCounter, graphRef.getIndex(platformId), graphRef.getIndex(exitId), generateDefaultAttributes(), currentAbsTime, PathStrategy::SHORTEST_TIME, true, &graphRef);
                        std::vector<int> path = graphRef.findPath(platformId, exitId, PathStrategy::SHORTEST_TIME);
                        if (!path.empty())
                        {
                            p.setPath(path);
                            train_passengers.push_back(p);
                            remaining--;
                        }
                        else
                        {
                            trainPassengerCounter--;
                            total_generated--;
                        }
                    }
                    else
                    {
                        trainPassengerCounter--;
                        total_generated--;
                    }
                }
            }
        }

        // 【关键修复3】：补全列车下车乘客的统计逻辑，防止数据凭空消失
        int trainCount = static_cast<int>(train_passengers.size());
        if (trainCount > 0)
        {
            // 【新增】获取当前所处的时间段，将列车乘客也归入对应时段档案
            const CrowdProfile *currentProfile = get_current_profile();
            if (currentProfile)
            {
                stats.profile_counts[currentProfile->name] += trainCount;
            }

            if (clock.is_weekday())
            {
                stats.weekday_total += trainCount;
                // 【修复】保持与入口进站完全一致的高峰/平峰判定逻辑 (rate >= 4.0 视为高峰)
                if (currentProfile && currentProfile->arrival_rate >= 4.0)
                {
                    stats.peak_total += trainCount;
                }
                else
                {
                    stats.offpeak_total += trainCount;
                }
            }
            else
            {
                stats.weekend_total += trainCount;
                stats.offpeak_total += trainCount;
            }
        }

        return train_passengers;
    }
    // [新增] 查找随机出口（优先选择同楼层出口）
    std::string findRandomExit(int floor = 0) const
    {
        std::vector<std::string> sameFloorExitIds;
        std::vector<std::string> otherFloorExitIds;

        for (const auto &node : graphRef.getAllNodes())
        {
            if (node->getTypeCode() == "EXIT")
            {
                // 如果指定了楼层，优先收集同楼层出口
                if (floor != 0 && node->getFloor() == floor)
                {
                    sameFloorExitIds.push_back(node->getId());
                }
                else
                {
                    otherFloorExitIds.push_back(node->getId());
                }
            }
        }

        // 优先选择同楼层出口
        std::vector<std::string> &exitIds = sameFloorExitIds.empty() ? otherFloorExitIds : sameFloorExitIds;

        if (!exitIds.empty())
        {
            std::uniform_int_distribution<> dist(0, exitIds.size() - 1);
            return exitIds[dist(rng)];
        }
        return "";
    }

    // [新增] 生成默认乘客属性
    PassengerAttributes generateDefaultAttributes() const
    {
        PassengerAttributes attrs;
        std::uniform_real_distribution<float> speed_dist(0.8f, 1.2f);
        std::uniform_real_distribution<float> patience_dist(0.5f, 1.0f);
        std::uniform_real_distribution<float> fam_dist(0.3f, 0.8f);
        std::bernoulli_distribution luggage_dist(0.1f); // 10%携带行李
        std::bernoulli_distribution purpose_dist(0.8f); // 80%通勤

        attrs.speed = speed_dist(rng);
        attrs.patience = patience_dist(rng);
        attrs.familiarity = fam_dist(rng);
        attrs.has_luggage = luggage_dist(rng);
        attrs.purpose = purpose_dist(rng) ? "commute" : "leisure";

        return attrs;
    }

    void initialize_default_schedule()
    {
        // Morning rush hour 7-9 AM on weekdays
        TimeSlot morning_peak;
        morning_peak.start_second = TimeSlot::time_to_seconds(7, 0);
        morning_peak.end_second = TimeSlot::time_to_seconds(9, 0);
        morning_peak.day_mask = TimeSlot::weekday_mask();
        morning_peak.profile.name = "工作日早高峰";
        morning_peak.profile.arrival_rate = 8.0;
        morning_peak.profile.familiarity_min = 0.8f;
        morning_peak.profile.familiarity_max = 1.0f;
        morning_peak.profile.patience_min = 0.4f;
        morning_peak.profile.patience_max = 0.7f;
        morning_peak.profile.luggage_prob = 0.15f;
        morning_peak.profile.commute_ratio = 0.95f;
        schedule.push_back(morning_peak);

        // Evening rush hour 5-7 PM on weekdays
        TimeSlot evening_peak;
        evening_peak.start_second = TimeSlot::time_to_seconds(17, 0);
        evening_peak.end_second = TimeSlot::time_to_seconds(19, 0);
        evening_peak.day_mask = TimeSlot::weekday_mask();
        evening_peak.profile.name = "工作日晚高峰";
        evening_peak.profile.arrival_rate = 7.0;
        evening_peak.profile.familiarity_min = 0.8f;
        evening_peak.profile.familiarity_max = 1.0f;
        evening_peak.profile.patience_min = 0.3f;
        evening_peak.profile.patience_max = 0.6f;
        evening_peak.profile.luggage_prob = 0.2f;
        evening_peak.profile.commute_ratio = 0.9f;
        schedule.push_back(evening_peak);

        // Off-peak weekday 9 AM - 5 PM
        TimeSlot weekday_offpeak;
        weekday_offpeak.start_second = TimeSlot::time_to_seconds(9, 0);
        weekday_offpeak.end_second = TimeSlot::time_to_seconds(17, 0);
        weekday_offpeak.day_mask = TimeSlot::weekday_mask();
        weekday_offpeak.profile.name = "工作日平峰";
        weekday_offpeak.profile.arrival_rate = 5.0;
        weekday_offpeak.profile.familiarity_min = 0.6f;
        weekday_offpeak.profile.familiarity_max = 0.9f;
        weekday_offpeak.profile.patience_min = 0.6f;
        weekday_offpeak.profile.patience_max = 0.9f;
        weekday_offpeak.profile.luggage_prob = 0.3f;
        weekday_offpeak.profile.commute_ratio = 0.5f;
        schedule.push_back(weekday_offpeak);

        // Weekend peak 10 AM - 8 PM
        TimeSlot weekend_peak;
        weekend_peak.start_second = TimeSlot::time_to_seconds(10, 0);
        weekend_peak.end_second = TimeSlot::time_to_seconds(20, 0);
        weekend_peak.day_mask = TimeSlot::weekend_mask();
        weekend_peak.profile.name = "周末高峰";
        weekend_peak.profile.arrival_rate = 5.0;
        weekend_peak.profile.familiarity_min = 0.3f;
        weekend_peak.profile.familiarity_max = 0.7f;
        weekend_peak.profile.patience_min = 0.7f;
        weekend_peak.profile.patience_max = 1.0f;
        weekend_peak.profile.luggage_prob = 0.5f;
        weekend_peak.profile.commute_ratio = 0.2f;
        schedule.push_back(weekend_peak);

        // Default off-peak
        TimeSlot default_slot;
        default_slot.start_second = 0;
        default_slot.end_second = 86400;
        default_slot.day_mask = TimeSlot::daily_mask();
        default_slot.profile.name = "默认平峰";
        default_slot.profile.arrival_rate = 4.0;
        schedule.push_back(default_slot);
    }

    const CrowdProfile *get_current_profile()
    {
        int weekday = clock.get_current_weekday();
        int sec_today = clock.get_seconds_today();

        for (const auto &slot : schedule)
        {
            bool day_match = (slot.day_mask & (1 << weekday));
            bool time_match = (sec_today >= slot.start_second &&
                               sec_today < slot.end_second);
            if (day_match && time_match)
            {
                return &slot.profile;
            }
        }

        return &schedule.back().profile;
    }
    // [新增] 生成入口进站乘客的方法（分离原有逻辑）
    std::vector<Passenger> generateEntryPassengers(double dt, int remaining = 9999)
    {
        std::vector<Passenger> new_passengers;
        const CrowdProfile *profile = get_current_profile();

        if (!profile || remaining <= 0)
            return new_passengers;

        double rate = profile->arrival_rate;

        std::poisson_distribution<int> dist(rate * dt);
        int count = dist(rng);
        count = std::min(count, remaining);

        if (count > 0)
        {
            size_t initial_size = new_passengers.size(); // 新增记录进入生成循环前的初始数量
            new_passengers.reserve(count);

            std::uniform_real_distribution<float> speed_dist(
                profile->speed_min, profile->speed_max);
            std::uniform_real_distribution<float> patience_dist(
                profile->patience_min, profile->patience_max);
            std::uniform_real_distribution<float> fam_dist(
                profile->familiarity_min, profile->familiarity_max);
            std::bernoulli_distribution luggage_dist(profile->luggage_prob);
            std::bernoulli_distribution purpose_dist(profile->commute_ratio);

            // 获取所有入口和出口节点
            std::vector<std::string> hallIds;
            std::vector<std::string> exitIds;
            std::vector<std::string> platformIds;
            for (const auto &node : graphRef.getAllNodes())
            {
                if (node->getTypeCode() == "HALL")
                {
                    hallIds.push_back(node->getId());
                }
                else if (node->getTypeCode() == "EXIT")
                {
                    exitIds.push_back(node->getId());
                }
                else if (node->getTypeCode() == "PLATFORM")
                {
                    platformIds.push_back(node->getId());
                }
            }

            if (hallIds.empty() || (exitIds.empty() && platformIds.empty()))
            {
                return new_passengers;
            }

            std::uniform_int_distribution<> hallDist(0, hallIds.size() - 1);
            std::uniform_int_distribution<> platformDist(0, platformIds.size() - 1);
            std::bernoulli_distribution direction_dist(0.6);

            for (int i = 0; i < count; ++i)
            {
                total_generated++;
                PassengerAttributes attrs;
                attrs.speed = speed_dist(rng);
                attrs.patience = patience_dist(rng);
                attrs.familiarity = fam_dist(rng);
                attrs.has_luggage = luggage_dist(rng);
                attrs.purpose = purpose_dist(rng) ? "commute" : "leisure";

                // 根据乘客特征和时间选择路径规划策略
                PathStrategy strategy = PathStrategy::MULTI_OBJECTIVE_OPTIMIZATION;

                // 如果有行李，选择最短距离
                if (attrs.has_luggage)
                {
                    strategy = PathStrategy::SHORTEST_DISTANCE;
                }
                // 如果是上班高峰期且是通勤目的，选择最短时间
                else if (purpose_dist(rng) && (clock.get_current_hour() >= 7 && clock.get_current_hour() <= 9))
                {
                    strategy = PathStrategy::SHORTEST_TIME;
                }

                std::string startId = hallIds[hallDist(rng)];

                // 获取起点楼层
                int startFloor = 0;
                AbstractNode *startNode = graphRef.getNode(startId);
                if (startNode)
                {
                    startFloor = startNode->getFloor();
                }

                std::string endId;
                bool headingToPlatform = direction_dist(rng) && !platformIds.empty();

                if (headingToPlatform)
                {
                    // 选择目标站台时，优先选择同楼层的站台
                    std::vector<std::string> sameFloorPlatforms;
                    std::vector<std::string> otherFloorPlatforms;
                    for (const std::string &pid : platformIds)
                    {
                        AbstractNode *pn = graphRef.getNode(pid);
                        if (pn && pn->getFloor() == startFloor)
                        {
                            sameFloorPlatforms.push_back(pid);
                        }
                        else
                        {
                            otherFloorPlatforms.push_back(pid);
                        }
                    }
                    std::vector<std::string> &selectedPlatforms = sameFloorPlatforms.empty() ? otherFloorPlatforms : sameFloorPlatforms;
                    std::uniform_int_distribution<> selectedPlatformDist(0, selectedPlatforms.size() - 1);
                    endId = selectedPlatforms[selectedPlatformDist(rng)];
                }
                else if (!exitIds.empty())
                {
                    // 选择出口时，优先选择同楼层或连接楼层的出口
                    // 1层和-1层通过楼梯连接，2层独立
                    std::vector<std::string> preferredExits;
                    std::vector<std::string> otherExits;
                    for (const std::string &eid : exitIds)
                    {
                        AbstractNode *en = graphRef.getNode(eid);
                        if (en)
                        {
                            int exitFloor = en->getFloor();
                            // 1层的乘客可以去1层或-1层的出口（通过楼梯连接）
                            if ((startFloor == 1 && (exitFloor == 1 || exitFloor == -1)) ||
                                (startFloor == 2 && exitFloor == 2))
                            {
                                preferredExits.push_back(eid);
                            }
                            else
                            {
                                otherExits.push_back(eid);
                            }
                        }
                    }
                    std::vector<std::string> &selectedExits = preferredExits.empty() ? otherExits : preferredExits;
                    std::uniform_int_distribution<> selectedExitDist(0, selectedExits.size() - 1);
                    endId = selectedExits[selectedExitDist(rng)];
                }
                else
                {
                    endId = platformIds[platformDist(rng)];
                    headingToPlatform = true;
                }

                new_passengers.emplace_back(total_generated, graphRef.getIndex(startId), graphRef.getIndex(endId),
                                            attrs, clock.get_total_seconds(), strategy, false, &graphRef);

                if (headingToPlatform)
                {
                    new_passengers.back().isFromTrain = false;
                    new_passengers.back().headingToPlatform = true;
                }

                // 计算路径
                std::vector<int> path = graphRef.findPath(startId, endId, strategy);

                if (!path.empty())
                {
                    new_passengers.back().setPath(path);
                }
                else
                {
                    // 【必须加上】路径不通，直接丢弃，防止内存越界污染整个系统
                    new_passengers.pop_back();
                    total_generated--;
                }
            }

            // Update stats（修复：使用实际成功加入的数量，而不是原始的count，防止路径失败被踢掉的乘客变成幽灵数据）
            int actual_count = static_cast<int>(new_passengers.size()) - initial_size;
            if (actual_count > 0)
            {
                stats.profile_counts[profile->name] += actual_count;
                if (clock.is_weekday())
                {
                    stats.weekday_total += actual_count;
                    if (rate >= 4.0)
                        stats.peak_total += actual_count;
                    else
                        stats.offpeak_total += actual_count;
                }
                else
                {
                    stats.weekend_total += actual_count;
                }
            }
        }

        return new_passengers;
    }
    // [修改] generate方法 - 雙重生成入口乘客和列车乘客
    std::vector<Passenger> generate(double dt, int currentOnlineCount = 0)
    {
        std::vector<Passenger> all_passengers;

        if (currentOnlineCount >= maxOnlinePassengers)
        {
            return all_passengers;
        }

        int remaining = maxOnlinePassengers - currentOnlineCount;

        // 生成入口进站乘客（原有逻辑）
        std::vector<Passenger> entry_passengers = generateEntryPassengers(dt, remaining);
        all_passengers.insert(all_passengers.end(),
                              entry_passengers.begin(), entry_passengers.end());

        remaining = maxOnlinePassengers - currentOnlineCount - static_cast<int>(all_passengers.size());
        if (remaining <= 0)
            return all_passengers;

        // 生成列车下车乘客（新增逻辑）
        std::vector<Passenger> train_passengers = generateTrainPassengers(dt, remaining);
        all_passengers.insert(all_passengers.end(),
                              train_passengers.begin(), train_passengers.end());

        return all_passengers;
    }

    void print_stats() const
    {
        /*std::cout << "\n========== 客流生成统计 ==========" << std::endl;
        std::cout << "总生成人数：" << total_generated << " (入口进站: " << (total_generated - trainPassengerCounter) << ", 列车下车: " << trainPassengerCounter << ")" << std::endl;
        std::cout << "工作日客流：" << stats.weekday_total << std::endl;
        std::cout << "周末客流：" << stats.weekend_total << std::endl;
        std::cout << "高峰时段客流：" << stats.peak_total << std::endl;
        std::cout << "平峰时段客流：" << stats.offpeak_total << std::endl;
        std::cout << "\n各时段分布：" << std::endl;
        for (const auto &pair : stats.profile_counts)
        {
            std::cout << "  " << pair.first << ": " << pair.second << " 人" << std::endl;
        }
        std::cout << "================================" << std::endl;*/
    }

    int get_total_generated() const { return total_generated; }
};

class SimulationStatistics
{
public:
    struct NodeStats
    {
        int max_load = 0;
        int total_visits = 0;
        double total_wait_time = 0.0;
        double avg_congestion = 0.0;
        int total_queue_time = 0; // 总排队时间
        int total_queued = 0;     // 总排队人数
        // [新增 F4 采样字段]
        double cumulative_congestion_sum = 0.0;
        long long cumulative_queue_length = 0;
        int sample_count = 0;
    };

    std::unordered_map<std::string, NodeStats> node_statistics;
    std::vector<double> passenger_times;
    int total_passengers = 0;

    void record_node_usage(const std::string &node_id, int load, double congestion, double wait_time = 0.0)
    {
        auto &stats = node_statistics[node_id];
        if (load > stats.max_load)
            stats.max_load = load;
        stats.total_visits++;
        stats.avg_congestion = (stats.avg_congestion * (stats.total_visits - 1) + congestion) / stats.total_visits;
        stats.total_wait_time += wait_time;
    }

    void record_queue_time(const std::string &node_id, int queue_time)
    {
        auto &stats = node_statistics[node_id];
        stats.total_queue_time += queue_time;
        stats.total_queued++;
    }

    void record_passenger_time(double time)
    {
        passenger_times.push_back(time);
        total_passengers++;
    }

    void print_analysis() const
    {
        // std::cout << "\n========== 仿真统计分析 ==========" << std::endl;

        //// Bottleneck identification
        // std::vector<std::pair<std::string, double>> congestion_ranking;
        // for (const auto &entry : node_statistics)
        //{
        //     congestion_ranking.push_back({entry.first, entry.second.avg_congestion});
        // }
        // std::sort(congestion_ranking.begin(), congestion_ranking.end(),
        //           [](const auto &a, const auto &b)
        //           { return a.second > b.second; });

        // std::cout << "拥堵排名前5的节点:" << std::endl;
        // for (int i = 0; i < std::min<int>(5, (int)congestion_ranking.size()); i++)
        //{
        //     const auto &item = congestion_ranking[i];
        //     std::cout << "  " << item.first << ": " << std::fixed << std::setprecision(2) << item.second << std::endl;
        // }

        //// Average travel time
        // if (!passenger_times.empty())
        //{
        //     double total_time = 0.0;
        //     for (double t : passenger_times)
        //         total_time += t;
        //     std::cout << "平均通行时间: " << std::fixed << std::setprecision(2)
        //               << total_time / passenger_times.size() << " 秒" << std::endl;
        // }

        //// Queue analysis
        // std::cout << "\n排队时间统计:" << std::endl;
        // std::vector<std::pair<std::string, double>> queue_ranking;
        // for (const auto &pair : node_statistics)
        //{
        //     const std::string &id = pair.first;
        //     const NodeStats &stats = pair.second;
        //     if (stats.total_queued > 0)
        //     {
        //         queue_ranking.push_back({id, static_cast<double>(stats.total_queue_time) / stats.total_queued});
        //     }
        // }
        // std::sort(queue_ranking.begin(), queue_ranking.end(),
        //           [](const auto &a, const auto &b)
        //           { return a.second > b.second; });

        // for (int i = 0; i < std::min<int>(5, (int)queue_ranking.size()); i++)
        //{
        //     const auto &item = queue_ranking[i];
        //     const std::string &id = item.first;
        //     double avg_queue_time = item.second;
        //     std::cout << "  " << id << ": " << std::fixed << std::setprecision(2) << avg_queue_time << " 秒" << std::endl;
        // }

        //// Peak load analysis
        // std::cout << "最大负载节点:" << std::endl;
        // std::vector<std::pair<std::string, int>> load_ranking;
        // for (const auto &entry : node_statistics)
        //{
        //     load_ranking.push_back({entry.first, entry.second.max_load});
        // }
        // std::sort(load_ranking.begin(), load_ranking.end(),
        //           [](const auto &a, const auto &b)
        //           { return a.second > b.second; });

        // for (int i = 0; i < std::min<int>(5, (int)load_ranking.size()); i++)
        //{
        //     const auto &item = load_ranking[i];
        //     std::cout << "  " << item.first << ": " << item.second << std::endl;
        // }

        // std::cout << "================================" << std::endl;
    }

    // [新增] F4 每秒采样函数，严格调用已有 API
    void record_tick_for_f4(const SubwayGraph &graph)
    {
        for (const auto &nodePtr : graph.getAllNodes())
        {
            if (!nodePtr)
                continue;

            std::string id = nodePtr->getId();
            auto &stats = node_statistics[id];

            // 严格调用 AbstractNode 的方法
            stats.cumulative_congestion_sum += nodePtr->getCongestionFactor();
            stats.cumulative_queue_length += nodePtr->getQueueLength();
            stats.sample_count++;
        }
    }

    // [新增] F4 数据导出函数，严格调用已有 API
    void exportF4Data(const std::string &nodeFile, const std::string &passengerFile, const SubwayGraph &graph) const
    {
        std::ofstream nFile(nodeFile);
        if (nFile.is_open())
        {
            nFile << "node_id,type,floor,x,y,avg_density,avg_queue,total_visits\n";
            for (const auto &pair : node_statistics)
            {
                const std::string &id = pair.first;
                const NodeStats &stats = pair.second;

                AbstractNode *node = graph.getNode(id);
                if (!node || stats.sample_count == 0)
                    continue;

                // 严格调用 AbstractNode 的属性获取方法
                nFile << id << ","
                      << node->getTypeCode() << ","
                      << node->getFloor() << ","
                      << node->getPos().x << ","
                      << node->getPos().y << ","
                      << (stats.cumulative_congestion_sum / stats.sample_count) << ","
                      << (static_cast<double>(stats.cumulative_queue_length) / stats.sample_count) << ","
                      << stats.total_visits << "\n";
            }
            nFile.close();
        }

        std::ofstream pFile(passengerFile);
        if (pFile.is_open())
        {
            pFile << "passenger_id,travel_time\n";
            for (size_t i = 0; i < passenger_times.size(); ++i)
            {
                pFile << i << "," << passenger_times[i] << "\n";
            }
            pFile.close();
        }
    }
};

class SimulationManager
{
private:
    VirtualClock clock;
    PassengerGenerator generator;
    std::vector<Passenger> passengers;
    std::vector<Passenger> completed_passengers;
    double sim_time;
    double dt;
    SubwayGraph &graph;
    SimulationStatistics statistics;
    // [新增] 统计列车相关数据
    int totalTrainPassengersGenerated = 0;
    int totalTrainArrivals = 0;

public:
    SimulationManager(SubwayGraph &g, int maxOnline = 2500)
        : clock(1, 7), generator(clock, g, maxOnline), sim_time(0.0), dt(1.0), graph(g)
    {
    }
    const std::vector<Passenger> &getPassengers() const
    {
        return passengers;
    }

    // [修改] run方法 - 处理列车乘客
    void run(int steps)
    {
        /*std::cout << "========== 地铁站人群流动仿真 ==========" << std::endl;
        std::cout << "仿真开始时间：" << clock.get_formatted_time() << std::endl;
        std::cout << "时间步长：" << dt << "秒" << std::endl;
        std::cout << "总步数：" << steps << std::endl;
        std::cout << "========================================\n"
                  << std::endl;*/

        for (int i = 0; i < steps; ++i)
        {
            // 1. Update graph nodes and edges FIRST (so nodes assign services before passengers check)
            for (auto &node : graph.getAllNodes())
            {
                node->update(dt);
            }
            graph.update(dt);

            // 2. Generate new passengers
            auto new_p = generator.generate(dt, static_cast<int>(passengers.size()));
            passengers.insert(passengers.end(),
                              std::make_move_iterator(new_p.begin()),
                              std::make_move_iterator(new_p.end()));

            // 3. Update all passenger states
            int replansThisFrame = 0;
            const int maxReplansPerFrame = 30;

            for (size_t idx = 0; idx < passengers.size();)
            {
                auto *current_node = graph.getNode(passengers[idx].current_node_id);

                if (!current_node)
                {
                    idx++;
                    continue;
                }

                int node_load = current_node->getCurrentLoad();
                int node_capacity = current_node->getCapacity();
                double node_service_time = current_node->getPassThroughTime();

                bool active = passengers[idx].update(dt, node_load, node_capacity, node_service_time, current_node, graph, replansThisFrame, maxReplansPerFrame);

                if (!active)
                {
                    if (passengers[idx].exit_time > 0)
                    {
                        statistics.record_passenger_time(passengers[idx].get_travel_time());
                    }

                    completed_passengers.push_back(std::move(passengers[idx]));
                    passengers[idx] = std::move(passengers.back());
                    passengers.pop_back();
                }
                else
                {
                    idx++;
                }
            }
            // [新增步骤] 在每一步结束时，调用 F4 采样函数
            statistics.record_tick_for_f4(graph);
            // 4. Record statistics periodically
            if (i % 10 == 0)
            {
                for (const auto &node : graph.getAllNodes())
                {
                    statistics.record_node_usage(
                        node->getId(),
                        node->getCurrentLoad(),
                        node->getCongestionFactor());

                    // Record queue information
                    if (node->getQueueLength() > 0)
                    {
                        statistics.record_queue_time(node->getId(), node->getQueueLength());
                    }
                }
            }

            // 5. Output status periodically
            if (i % 100 == 0)
            {
                std::cout << "[" << clock.get_formatted_time() << "] "
                          << "在线人数：" << passengers.size()
                          << " | 累计生成：" << generator.get_total_generated()
                          << " | 已完成：" << completed_passengers.size()
                          << std::endl;

                // Show sample passenger states
                if (!passengers.empty())
                {
                    std::cout << "  乘客状态示例：";
                    int count = 0;
                    for (const auto &p : passengers)
                    {
                        if (count++ >= 5)
                            break;
                        std::cout << p.get_state_string() << " ";
                    }
                    std::cout << std::endl;
                }
            }

            // 6. Update time
            sim_time += dt;
            clock.update(dt); // 你的源码中使用的确实是 update(dt)
        }

        // Output final report
        /* std::cout << "\n========== 仿真结束 ==========" << std::endl;
         std::cout << "结束时间：" << clock.get_formatted_time() << std::endl;
         std::cout << "仿真总时长：" << sim_time / 3600 << " 小时" << std::endl;
         std::cout << "累计服务总人数：" << passengers.size() + completed_passengers.size() << std::endl;
         std::cout << "完成通行人数：" << completed_passengers.size() << std::endl;*/

        // Calculate average travel time
        /*if (!completed_passengers.empty())
        {
            double total_time = 0.0;
            for (const auto &p : completed_passengers)
            {
                total_time += p.get_travel_time();
            }
            std::cout << "平均通行时间：" << std::fixed << std::setprecision(2)
                      << total_time / completed_passengers.size() << " 秒" << std::endl;
        }*/

        generator.print_stats();
        statistics.print_analysis();

        // 在 SimulationManager::run(int sim_time) 函数的末尾
        // sim_time 本身就是仿真的总秒数（总步数）
        // f4部分
        statistics.exportF4Data("f4_nodes.csv", "f4_passengers.csv", graph);
    }
};

// 窗口尺寸定义
const int WINDOW_WIDTH = 1500;
const int WINDOW_HEIGHT = 1500;

// 应用状态枚举
enum class AppState
{
    Welcome,
    Simulation
};

// 按钮结构体
struct Button
{
    int x, y, width, height;
    std::wstring text;

    void draw() const
    {
        setfillcolor(LIGHTGRAY);
        setlinecolor(BLACK);
        fillrectangle(x, y, x + width, y + height);
        rectangle(x, y, x + width, y + height);

        settextcolor(BLACK);
        LOGFONT f = {};
        f.lfHeight = 20;
        f.lfWeight = FW_BOLD;
        wcscpy_s(f.lfFaceName, L"微软雅黑");
        settextstyle(&f);

        int tw = textwidth(text.c_str());
        int th = textheight(text.c_str());
        outtextxy(x + (width - tw) / 2, y + (height - th) / 2, text.c_str());
    }

    bool isHovered(int mx, int my) const
    {
        return mx >= x && mx <= x + width && my >= y && my <= y + height;
    }
};

// 地铁仿真界面类
class SubwaySimulationUI
{
public:
    SubwaySimulationUI(SubwayGraph &g, SimulationManager &sm);
    ~SubwaySimulationUI();
    void run();

private:
    void processEvents();
    void render();
    void floorUp();
    void floorDown();
    void zoomIn(int mx, int my);
    void zoomOut(int mx, int my);
    void drawMap();
    void drawPassengers();

    SubwayGraph &graph;
    SimulationManager &simManager;

    AppState currentState;
    int currentFloor;
    float viewScale;
    float offsetX;
    float offsetY;
    bool isDragging;
    int dragStartX;
    int dragStartY;
    bool isRunning;
    int simulationStep;     // 当前仿真步数
    int maxSimulationSteps; // 最大仿真步数（120步=2小时）

    Button startBtn;
    Button upBtn;
    Button downBtn;
};

SubwaySimulationUI::SubwaySimulationUI(SubwayGraph &g, SimulationManager &sm)
    : graph(g), simManager(sm),
      currentState(AppState::Welcome),
      currentFloor(1),
      viewScale(1.0f),
      offsetX(0.0f),
      offsetY(0.0f),
      isDragging(false),
      dragStartX(0),
      dragStartY(0),
      isRunning(true),
      simulationStep(0),
      maxSimulationSteps(14400) // 2小时，每步1分钟
{
    initgraph(WINDOW_WIDTH, WINDOW_HEIGHT, EX_SHOWCONSOLE);
    BeginBatchDraw();

    startBtn = {(WINDOW_WIDTH - 200) / 2, WINDOW_HEIGHT / 2 + 50, 200, 50, L"开始仿真"};

    int btnX = WINDOW_WIDTH - 130;
    upBtn = {btnX, 150, 110, 40, L"▲ 上层"};
    downBtn = {btnX, 200, 110, 40, L"▼ 下层"};
}

SubwaySimulationUI::~SubwaySimulationUI()
{
    EndBatchDraw();
    closegraph();
}

void SubwaySimulationUI::run()
{
    while (isRunning)
    {
        processEvents();
        render();
        Sleep(16);
    }
}

void SubwaySimulationUI::processEvents()
{
    ExMessage msg;
    while (peekmessage(&msg, EM_MOUSE | EM_KEY))
    {
        if (msg.message == WM_KEYDOWN)
        {
            if (msg.vkcode == VK_ESCAPE || msg.vkcode == 'Q' || msg.vkcode == 'q')
            {
                isRunning = false;
            }
        }

        if (msg.message == WM_LBUTTONDOWN)
        {
            if (currentState == AppState::Welcome)
            {
                if (startBtn.isHovered(msg.x, msg.y))
                {
                    currentState = AppState::Simulation;
                }
            }
            else if (currentState == AppState::Simulation)
            {
                if (upBtn.isHovered(msg.x, msg.y))
                    floorUp();
                if (downBtn.isHovered(msg.x, msg.y))
                    floorDown();
            }
        }
    }

    if (msg.message == WM_RBUTTONDOWN)
    {
        if (currentState == AppState::Simulation)
        {
            isDragging = true;
            dragStartX = msg.x;
            dragStartY = msg.y;
        }
    }
    else if (msg.message == WM_RBUTTONUP)
    {
        isDragging = false;
    }
    else if (msg.message == WM_MOUSEMOVE)
    {
        if (isDragging && currentState == AppState::Simulation)
        {
            offsetX += (msg.x - dragStartX);
            offsetY += (msg.y - dragStartY);
            dragStartX = msg.x;
            dragStartY = msg.y;
        }
    }

    if (msg.message == WM_MOUSEWHEEL)
    {
        if (currentState == AppState::Simulation)
        {
            if (msg.wheel > 0)
            {
                zoomIn(msg.x, msg.y);
            }
            else if (msg.wheel < 0)
            {
                zoomOut(msg.x, msg.y);
            }
        }
    }
}

void SubwaySimulationUI::render()
{
    setbkcolor(WHITE);
    cleardevice();

    if (currentState == AppState::Welcome)
    {
        settextcolor(BLACK);
        LOGFONT titleFont = {};
        titleFont.lfHeight = 50;
        titleFont.lfWeight = FW_HEAVY;
        wcscpy_s(titleFont.lfFaceName, L"微软雅黑");
        settextstyle(&titleFont);

        std::wstring titleText = L"地铁站人群流动仿真系统";
        int tw = textwidth(titleText.c_str());
        outtextxy((WINDOW_WIDTH - tw) / 2, WINDOW_HEIGHT / 2 - 100, titleText.c_str());

        startBtn.draw();
    }
    else if (currentState == AppState::Simulation)
    {
        // 运行仿真一步
        simManager.run(1);
        simulationStep++;

        // 检查是否达到最大步数
        if (simulationStep >= maxSimulationSteps)
        {
            isRunning = false;
        }

        // 绘制地图结构
        drawMap();
        // 绘制乘客
        drawPassengers();

        settextcolor(BLACK);
        LOGFONT mainFont = {};
        mainFont.lfHeight = 28;
        mainFont.lfWeight = FW_BOLD;
        wcscpy_s(mainFont.lfFaceName, L"微软雅黑");
        settextstyle(&mainFont);

        outtextxy(20, 20, L"【地铁站流动实时监控面板】");

        std::wstring floorText = L"当前楼层: " + std::to_wstring(currentFloor) + L" 层";
        outtextxy(20, 60, floorText.c_str());

        wchar_t scaleBuffer[32];
        swprintf(scaleBuffer, 32, L"视图倍率: %.1fx", viewScale);
        outtextxy(20, 100, scaleBuffer);

        upBtn.draw();
        downBtn.draw();
    }

    FlushBatchDraw();
}

void SubwaySimulationUI::floorUp()
{
    if (currentFloor == -1)
    {
        currentFloor = 1;
    }
    else if (currentFloor < 7)
    {
        currentFloor++;
    }
}

void SubwaySimulationUI::floorDown()
{
    if (currentFloor == 1)
    {
        currentFloor = -1;
    }
    else if (currentFloor > -3)
    {
        currentFloor--;
    }
}

void SubwaySimulationUI::zoomIn(int mx, int my)
{
    float oldScale = viewScale;
    float newScale = viewScale + 0.1f;
    viewScale = (newScale < 3.0f) ? newScale : 3.0f;

    if (viewScale != oldScale)
    {
        float logicalX = (mx - offsetX) / oldScale;
        float logicalY = (my - offsetY) / oldScale;
        offsetX = mx - logicalX * viewScale;
        offsetY = my - logicalY * viewScale;
    }
}

void SubwaySimulationUI::zoomOut(int mx, int my)
{
    float oldScale = viewScale;
    float newScale = viewScale - 0.1f;
    viewScale = (newScale > 0.5f) ? newScale : 0.5f;

    if (viewScale != oldScale)
    {
        float logicalX = (mx - offsetX) / oldScale;
        float logicalY = (my - offsetY) / oldScale;
        offsetX = mx - logicalX * viewScale;
        offsetY = my - logicalY * viewScale;
    }
}

void SubwaySimulationUI::drawMap()
{
    // 绘制边
    setlinecolor(LIGHTGRAY);
    for (size_t i = 0; i < graph.getAllNodes().size(); ++i)
    {
        const auto &fromNode = graph.getAllNodes()[i];
        if (fromNode->getFloor() == currentFloor)
        {
            const auto &edges = graph.getNeighbors(i);
            for (const auto &edge : edges)
            {
                int toIdx = edge.getToIndex();
                const auto &toNode = graph.getNode(toIdx);
                if (toNode && toNode->getFloor() == currentFloor)
                {
                    MYPOINT fromPos = fromNode->getPos();
                    MYPOINT toPos = toNode->getPos();
                    float rx1 = fromPos.x * viewScale + offsetX;
                    float ry1 = fromPos.y * viewScale + offsetY;
                    float rx2 = toPos.x * viewScale + offsetX;
                    float ry2 = toPos.y * viewScale + offsetY;

                    // 计算方向向量
                    float dx = rx2 - rx1;
                    float dy = ry2 - ry1;
                    float length = sqrt(dx * dx + dy * dy);

                    if (length > 0)
                    {
                        // 单位方向向量
                        float ux = dx / length;
                        float uy = dy / length;

                        // 垂直单位向量（用于偏移）
                        float vx = -uy;
                        float vy = ux;

                        // 偏移量（根据视图比例调整）
                        float offset = 5.0f * viewScale;

                        // 计算两条平行线的起点和终点
                        float rx1a = rx1 + vx * offset;
                        float ry1a = ry1 + vy * offset;
                        float rx2a = rx2 + vx * offset;
                        float ry2a = ry2 + vy * offset;

                        float rx1b = rx1 - vx * offset;
                        float ry1b = ry1 - vy * offset;
                        float rx2b = rx2 - vx * offset;
                        float ry2b = ry2 - vy * offset;

                        // 绘制两条平行线
                        line(static_cast<int>(rx1a), static_cast<int>(ry1a), static_cast<int>(rx2a), static_cast<int>(ry2a));
                        line(static_cast<int>(rx1b), static_cast<int>(ry1b), static_cast<int>(rx2b), static_cast<int>(ry2b));
                    }
                }
            }
        }
    }

    // 绘制节点
    setbkmode(TRANSPARENT);
    settextcolor(BLACK);
    LOGFONT f = {};
    f.lfHeight = static_cast<long>(20 * viewScale);
    f.lfWeight = FW_NORMAL;
    wcscpy_s(f.lfFaceName, L"微软雅黑");
    settextstyle(&f);

    for (const auto &node : graph.getAllNodes())
    {
        if (node->getFloor() == currentFloor)
        {
            MYPOINT pos = node->getPos();
            float rx = pos.x * viewScale + offsetX;
            float ry = pos.y * viewScale + offsetY;

            setlinecolor(BLACK);
            setfillcolor(WHITE);

            // 根据节点类型绘制不同的形状
            std::string typeCode = node->getTypeCode();
            if (typeCode == "HALL")
            {
                // 站厅：横向大号矩形
                float rw = 120 * viewScale;
                float rh = 40 * viewScale;
                int left = static_cast<int>(rx - rw / 2);
                int top = static_cast<int>(ry - rh / 2);
                int right = static_cast<int>(rx + rw / 2);
                int bottom = static_cast<int>(ry + rh / 2);
                fillrectangle(left, top, right, bottom);
                rectangle(left, top, right, bottom);
            }
            else if (typeCode == "PLATFORM")
            {
                // 站台：纵向长方形（更长）
                float rw = 40 * viewScale;
                float rh = 150 * viewScale;
                int left = static_cast<int>(rx - rw / 2);
                int top = static_cast<int>(ry - rh / 2);
                int right = static_cast<int>(rx + rw / 2);
                int bottom = static_cast<int>(ry + rh / 2);
                fillrectangle(left, top, right, bottom);
                rectangle(left, top, right, bottom);
            }
            else if (typeCode == "TICKET")
            {
                // 售票区：圆角小矩形
                float rw = 40 * viewScale;
                float rh = 25 * viewScale;
                int left = static_cast<int>(rx - rw / 2);
                int top = static_cast<int>(ry - rh / 2);
                int right = static_cast<int>(rx + rw / 2);
                int bottom = static_cast<int>(ry + rh / 2);
                fillroundrect(left, top, right, bottom, 10, 10);
                roundrect(left, top, right, bottom, 10, 10);
            }
            else if (typeCode == "GATE")
            {
                // 闸机：横向椭圆形
                int rxRadius = static_cast<int>(25 * viewScale);
                int ryRadius = static_cast<int>(12 * viewScale);
                int left = static_cast<int>(rx - rxRadius);
                int top = static_cast<int>(ry - ryRadius);
                int right = static_cast<int>(rx + rxRadius);
                int bottom = static_cast<int>(ry + ryRadius);
                fillellipse(left, top, right, bottom);
                ellipse(left, top, right, bottom);
            }
            else if (typeCode == "SECURITY")
            {
                // 安检区：菱形
                POINT pts[4];
                int rW = static_cast<int>(30 * viewScale);
                int rH = static_cast<int>(20 * viewScale);
                pts[0].x = static_cast<int>(rx);
                pts[0].y = static_cast<int>(ry - rH);
                pts[1].x = static_cast<int>(rx + rW);
                pts[1].y = static_cast<int>(ry);
                pts[2].x = static_cast<int>(rx);
                pts[2].y = static_cast<int>(ry + rH);
                pts[3].x = static_cast<int>(rx - rW);
                pts[3].y = static_cast<int>(ry);
                fillpolygon(pts, 4);
                polygon(pts, 4);
            }
            else if (typeCode == "EXIT")
            {
                // 出口：正圆形
                int radius = static_cast<int>(20 * viewScale);
                fillcircle(static_cast<int>(rx), static_cast<int>(ry), radius);
                circle(static_cast<int>(rx), static_cast<int>(ry), radius);
            }
            else if (typeCode == "STAIR")
            {
                // 楼梯：三角形
                POINT pts[3];
                int rW = static_cast<int>(30 * viewScale);
                int rH = static_cast<int>(30 * viewScale);
                pts[0].x = static_cast<int>(rx);
                pts[0].y = static_cast<int>(ry - rH / 2);
                pts[1].x = static_cast<int>(rx + rW);
                pts[1].y = static_cast<int>(ry + rH / 2);
                pts[2].x = static_cast<int>(rx - rW);
                pts[2].y = static_cast<int>(ry + rH / 2);
                fillpolygon(pts, 3);
                polygon(pts, 3);
            }

            // 绘制节点类型标签（使用英文首字母缩写）
            std::wstring label(typeCode.begin(), typeCode.end());
            int tw = textwidth(label.c_str());
            int th = textheight(label.c_str());
            outtextxy(static_cast<int>(rx) - tw / 2, static_cast<int>(ry) - th / 2, label.c_str());
        }
    }
}

void SubwaySimulationUI::drawPassengers()
{
    setfillcolor(RED);
    setlinecolor(RED);

    const float baseRadius = 5.0f; // 乘客基础像素大小

    // 从 SimulationManager 中获取乘客信息
    const auto &passengers = simManager.getPassengers();
    for (const auto &passenger : passengers)
    {
        if (passenger.getFloor() == currentFloor)
        {
            // 检查乘客是否在边上移动
            if (passenger.getState() == PassengerState::IN_TRANSIT)
            {
                // 计算乘客在边上的位置
                int fromNodeId = passenger.getCurrentEdgeFrom();
                int toNodeId = passenger.getCurrentEdgeTo();
                double transitProgress = passenger.getTransitProgress();

                if (fromNodeId != -1 && toNodeId != -1)
                {
                    const auto &fromNode = graph.getNode(fromNodeId);
                    const auto &toNode = graph.getNode(toNodeId);

                    if (fromNode && toNode)
                    {
                        MYPOINT fromPos = fromNode->getPos();
                        MYPOINT toPos = toNode->getPos();

                        // 线性插值计算当前位置，确保transitProgress在0-1之间
                        double clampedProgress = std::max(0.0, std::min(1.0, transitProgress));
                        float x = fromPos.x + (toPos.x - fromPos.x) * clampedProgress;
                        float y = fromPos.y + (toPos.y - fromPos.y) * clampedProgress;

                        float renderX = x * viewScale + offsetX;
                        float renderY = y * viewScale + offsetY;
                        float renderRadius = baseRadius * viewScale;
                        solidcircle(static_cast<int>(renderX), static_cast<int>(renderY), static_cast<int>(renderRadius));
                    }
                }
            }
            else
            {
                // 乘客在节点内，使用节点位置
                MYPOINT pos = passenger.getPosition();
                float renderX = pos.x * viewScale + offsetX;
                float renderY = pos.y * viewScale + offsetY;
                float renderRadius = baseRadius * viewScale;
                solidcircle(static_cast<int>(renderX), static_cast<int>(renderY), static_cast<int>(renderRadius));
            }
        }
    }
}

// Main function to demonstrate the system
int main()
{
    std::cout << "地铁站人群流动仿真系统启动..." << std::endl;

    // 创建地铁图
    SubwayGraph graph;

    if (!graph.loadFromCSV("subway_config.csv"))
    {
        std::cerr << "错误：无法加载配置文件！请检查文件是否在 Debug 目录下。" << std::endl;
        return -1; // 如果加载失败，直接退出
    }

    std::cout << "成功加载 " << graph.getAllNodes().size() << " 个节点。" << std::endl;

    // --- 可视化检查 ---
    // 注意：如果节点太多（200个），visualize() 打印到控制台会非常慢且刷屏
    // 建议调试时先注释掉 visualize
    // graph.visualize();

    // 创建仿真管理器
    SimulationManager sim(graph);

    // 创建并运行界面（仿真在GUI循环中执行，5分钟后自动退出）
    SubwaySimulationUI ui(graph, sim);
    ui.run();

    return 0;
}