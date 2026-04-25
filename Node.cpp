#include "Node.h"
#include "Edge.h"
#include <iostream>
#include <cmath>

AbstractNode::AbstractNode(const std::string& nodeId, int nodeFloor, POINT position, int cap,
    double vel, double sensitivity, double sRate, int maxServ)
    : id(nodeId), floor(nodeFloor), pos(position), capacity(cap), currentLoad(0),
    baseVelocity(vel), congestionFactor(0.0), congestionSensitivity(sensitivity),
    serviceRate(sRate), serviceTimer(0.0), maxSimultaneousServices(maxServ),
    gridWidth(20), gridHeight(20), cellSize(0.5) {
    initializeGrid();
}

void AbstractNode::initializeGrid() {
    occupancyGrid.assign(gridWidth, std::vector<int>(gridHeight, 0));
    for (int i = 0; i < gridWidth; ++i) {
        occupancyGrid[i][0] = -1;
        occupancyGrid[i][gridHeight - 1] = -1;
    }
    for (int j = 0; j < gridHeight; ++j) {
        occupancyGrid[0][j] = -1;
        occupancyGrid[gridWidth - 1][j] = -1;
    }
}

bool AbstractNode::isCellOccupied(int x, int y) const {
    if (!isCellValid(x, y)) return true;
    return occupancyGrid[x][y] != 0;
}

bool AbstractNode::isCellObstacle(int x, int y) const {
    if (!isCellValid(x, y)) return true;
    return occupancyGrid[x][y] == -1;
}

bool AbstractNode::occupyCell(int x, int y, int passengerId) {
    if (!isCellValid(x, y) || isCellOccupied(x, y)) return false;
    occupancyGrid[x][y] = passengerId;
    onPassengerArrive();
    return true;
}

bool AbstractNode::releaseCell(int x, int y) {
    if (!isCellValid(x, y) || occupancyGrid[x][y] <= 0) return false;
    occupancyGrid[x][y] = 0;
    onPassengerLeave();
    return true;
}

bool AbstractNode::moveCell(int from_x, int from_y, int to_x, int to_y, int passengerId) {
    if (!isCellValid(to_x, to_y) || isCellOccupied(to_x, to_y)) return false;
    if (!isCellValid(from_x, from_y) || occupancyGrid[from_x][from_y] != passengerId) return false;
    occupancyGrid[from_x][from_y] = 0;
    occupancyGrid[to_x][to_y] = passengerId;
    return true;
}

// 排队相关方法
bool AbstractNode::canJoinQueue() const {
    // 只用队列长度限制，绝对不能用拥堵因子锁死入口
    // 否则任何 >= 0.9 的拥堵都会导致队列永久关闭，形成死循环
    int maxQueueSize = static_cast<int>(capacity * 0.8);
    if (maxQueueSize < 1) maxQueueSize = 1;
    return waitingQueue.size() < maxQueueSize;
}

bool AbstractNode::joinQueue(int passengerId) {
    if (canJoinQueue()) { waitingQueue.push(passengerId); return true; }
    return false;
}

int AbstractNode::serveNextPassenger() {
    if (!waitingQueue.empty() && servingPassengers.size() < maxSimultaneousServices) {
        int passengerId = waitingQueue.front();
        waitingQueue.pop();
        servingPassengers.insert(passengerId);
        return passengerId;
    }
    return -1;
}

void AbstractNode::onPassengerArrive() {
    if (currentLoad < capacity) { ++currentLoad; updateCongestionFactor(); }
}

void AbstractNode::onPassengerLeave() {
    if (currentLoad > 0) { --currentLoad; updateCongestionFactor(); }
}

void AbstractNode::updateCongestionFactor() {
    double ratio = capacity > 0 ? static_cast<double>(currentLoad) / capacity : 0.0;
    double temp = ratio * congestionSensitivity;
    congestionFactor = (temp < 1.0) ? temp : 1.0;
}

bool AbstractNode::canExit(const AbstractNode* nextNode, const Edge* connectingEdge) const {
    if (nextNode && !nextNode->canEnter()) return false;
    if (connectingEdge && !connectingEdge->canEnter()) return false;
    return servingPassengers.size() < maxSimultaneousServices * 0.9;
}

double AbstractNode::getPassThroughTime() const {
    double effectiveVel = getVelocity();
    if (effectiveVel > 0.001) return (1.0 / effectiveVel) * (1.0 + congestionFactor);
    return 999.0;
}

void AbstractNode::assignServingPassengers() {
    while (waitingQueue.size() > 0 && servingPassengers.size() < maxSimultaneousServices) {
        int pid = serveNextPassenger();
        if (pid == -1) break;
    }
}

double AbstractNode::getServiceInterval() const { return 1.0 / serviceRate; }

void AbstractNode::update(double deltaTime) {
    assignServingPassengers();
    updateCongestionFactor();
}

std::map<std::string, std::string> AbstractNode::toProperties() const {
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

void AbstractNode::fromProperties(const std::map<std::string, std::string>& props) {
    auto get = [&](const std::string& key) -> std::string {
        auto it = props.find(key);
        return it != props.end() ? it->second : "";
    };
    if (!get("id").empty()) id = get("id");
    if (!get("floor").empty()) setFloor(safeStoi(get("floor")));
    if (!get("x").empty() && !get("y").empty()) pos = { safeStoi(get("x")), safeStoi(get("y")) };
    if (!get("capacity").empty()) setCapacity(safeStoi(get("capacity")));
    if (!get("baseVelocity").empty()) setBaseVelocity(safeStod(get("baseVelocity")));
    if (!get("congestionFactor").empty()) congestionFactor = safeStod(get("congestionFactor"));
    if (!get("congestionSensitivity").empty()) setCongestionSensitivity(safeStod(get("congestionSensitivity")));
    if (!get("currentLoad").empty()) currentLoad = safeStoi(get("currentLoad"));
}

void HallNode::render() const {
    std::cout << "\u7ad9\u5385: " << id << ", \u4eba\u6570: " << currentLoad << ", \u62e5\u5835: " << congestionFactor
        << ", \u961f\u5217: " << waitingQueue.size() << std::endl;
}

void SecurityNode::render() const {
    std::cout << "\u5b89\u68c0: " << id << ", \u4eba\u6570: " << currentLoad << ", \u62e5\u5835: " << congestionFactor
        << ", \u961f\u5217: " << waitingQueue.size() << std::endl;
}

std::map<std::string, std::string> SecurityNode::toProperties() const {
    auto props = AbstractNode::toProperties();
    props["scannerCount"] = std::to_string(scannerCount);
    props["checkTimePerPerson"] = std::to_string(checkTimePerPerson);
    props["hasBannedItem"] = hasBannedItem ? "1" : "0";
    return props;
}

void TicketNode::render() const {
    std::cout << "\u552e\u7968: " << id << ", \u4eba\u6570: " << currentLoad << ", \u62e5\u5835: " << congestionFactor
        << ", \u961f\u5217: " << waitingQueue.size() << std::endl;
}

std::map<std::string, std::string> TicketNode::toProperties() const {
    auto props = AbstractNode::toProperties();
    props["windowCount"] = std::to_string(windowCount);
    props["buyTimePerPerson"] = std::to_string(buyTimePerPerson);
    props["hasAutoMachine"] = hasAutoMachine ? "1" : "0";
    return props;
}

void GateNode::render() const {
    std::cout << "\u95f8\u673a: " << id << ", \u4eba\u6570: " << currentLoad << ", \u62e5\u5835: " << congestionFactor
        << ", \u961f\u5217: " << waitingQueue.size() << std::endl;
}

std::map<std::string, std::string> GateNode::toProperties() const {
    auto props = AbstractNode::toProperties();
    props["gateCount"] = std::to_string(gateCount);
    props["isBidirectional"] = isBidirectional ? "1" : "0";
    return props;
}

void ExitNode::render() const {
    std::cout << "\u51fa\u53e3: " << id << ", \u4eba\u6570: " << currentLoad << ", \u62e5\u5835: " << congestionFactor
        << ", \u961f\u5217: " << waitingQueue.size() << std::endl;
}

std::map<std::string, std::string> ExitNode::toProperties() const {
    auto props = AbstractNode::toProperties();
    props["exitName"] = exitName;
    props["connectedStreet"] = connectedStreet;
    props["isOneWay"] = isOneWay ? "1" : "0";
    props["totalExits"] = std::to_string(totalExits);
    return props;
}

void PlatformNode::update(double deltaTime) {
    handleTrainArrival(deltaTime);
    AbstractNode::update(deltaTime);
}

void PlatformNode::handleTrainArrival(double deltaTime) {
    if (doorOpenTimer > 0) {
        doorOpenTimer -= deltaTime;
        isTrainArriving = true;
    } else {
        nextTrainIn -= deltaTime;
        if (nextTrainIn <= 0) {
            isTrainArriving = true;
            doorOpenTimer = DOOR_OPEN_DURATION;
            nextTrainIn = 120.0;
        } else {
            isTrainArriving = false;
        }
    }
}

void PlatformNode::render() const {
    std::cout << "\u7ad9\u53f0: " << id << " (" << lineName << "), \u4eba\u6570: " << currentLoad << ", \u62e5\u5835: " << congestionFactor
        << ", \u961f\u5217: " << waitingQueue.size() << std::endl;
}

std::map<std::string, std::string> PlatformNode::toProperties() const {
    auto props = AbstractNode::toProperties();
    props["lineName"] = lineName;
    props["direction"] = std::to_string(direction);
    props["waitCap"] = std::to_string(waitCap);
    props["hasScreenDoor"] = hasScreenDoor ? "1" : "0";
    props["nextTrainIn"] = std::to_string(nextTrainIn);
    return props;
}

void PlatformNode::fromProperties(const std::map<std::string, std::string>& props) {
    AbstractNode::fromProperties(props);
    auto get = [&](const std::string& key) -> std::string {
        auto it = props.find(key);
        return it != props.end() ? it->second : "";
    };
    if (!get("lineName").empty()) lineName = get("lineName");
    if (!get("direction").empty()) direction = safeStoi(get("direction"));
    if (!get("waitCap").empty()) waitCap = safeStoi(get("waitCap"));
    if (!get("hasScreenDoor").empty()) hasScreenDoor = (get("hasScreenDoor") == "1");
    if (!get("nextTrainIn").empty()) nextTrainIn = safeStod(get("nextTrainIn"));
}

void StairNode::render() const {
    std::cout << "\u697c\u68af: " << id << ", \u4eba\u6570: " << currentLoad << ", \u62e5\u5835: " << congestionFactor
        << ", \u961f\u5217: " << waitingQueue.size() << std::endl;
}

std::map<std::string, std::string> StairNode::toProperties() const {
    auto props = AbstractNode::toProperties();
    props["stepCount"] = std::to_string(stepCount);
    return props;
}

std::unique_ptr<AbstractNode> NodeFactory::createNode(const std::string& typeCode, const std::map<std::string, std::string>& props) {
    std::string id = props.at("id");
    int floor = safeStoi(props.at("floor"));
    POINT pos = { safeStoi(props.at("x")), safeStoi(props.at("y")) };
    int cap = safeStoi(props.at("capacity"));
    double vel = safeStod(props.at("baseVelocity"));
    double sens = safeStod(props.at("sensitivity"));

    if (typeCode == "HALL") return std::make_unique<HallNode>(id, floor, pos, cap, vel, sens);
    if (typeCode == "SECURITY") {
        int scanners = safeStoi(props.at("scannerCount"));
        double t = safeStod(props.at("checkTimePerPerson"));
        bool hasBanned = (props.at("hasBannedItem") == "1");
        return std::make_unique<SecurityNode>(id, floor, pos, cap, vel, sens, scanners, t, hasBanned);
    }
    if (typeCode == "TICKET") {
        int win = safeStoi(props.at("windowCount"));
        double t = safeStod(props.at("buyTimePerPerson"));
        bool hasAuto = (props.at("hasAutoMachine") == "1");
        return std::make_unique<TicketNode>(id, floor, pos, cap, vel, sens, win, t, hasAuto);
    }
    if (typeCode == "GATE") {
        int g = safeStoi(props.at("gateCount"));
        bool bidir = (props.at("isBidirectional") == "1");
        return std::make_unique<GateNode>(id, floor, pos, cap, vel, sens, g, bidir);
    }
    if (typeCode == "EXIT") {
        int total = safeStoi(props.at("totalExits"));
        return std::make_unique<ExitNode>(id, floor, pos, cap, vel, sens, props.at("exitName"), props.at("connectedStreet"), props.at("isOneWay") == "1", total);
    }
    if (typeCode == "PLATFORM") {
        return std::make_unique<PlatformNode>(id, floor, pos, cap, vel, sens,
            props.at("lineName"), safeStoi(props.at("direction")),
            safeStoi(props.at("waitCap")), props.at("hasScreenDoor") == "1", safeStod(props.at("nextTrainIn")));
    }
    if (typeCode == "STAIR") {
        return std::make_unique<StairNode>(id, floor, pos, cap, vel, sens, safeStoi(props.at("stepCount")));
    }
    return nullptr;
}