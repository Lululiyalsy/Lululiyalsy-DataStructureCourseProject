#include "SubwayGraph.h"
#include "Node.h"
#include "Edge.h"
#include "Common.h"
#include "Enums.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

SubwayGraph::SubwayGraph() = default;

SubwayGraph::~SubwayGraph() = default;

void SubwayGraph::ensurePathBuffers() const {
    size_t n = nodes_.size();
    if (pathDist_.size() != n) {
        pathDist_.resize(n);
        pathPrev_.resize(n);
        pathVisited_.resize(n);
        estVisited_.resize(n);
        estLevel_.resize(n);
        congestionCache_.resize(n, 0.0);
    }
}

void SubwayGraph::resetPathBuffers(double infVal) const {
    ensurePathBuffers();
    std::fill(pathDist_.begin(), pathDist_.end(), infVal);
    std::fill(pathPrev_.begin(), pathPrev_.end(), -1);
    std::fill(pathVisited_.begin(), pathVisited_.end(), false);
}

void SubwayGraph::rebuildCongestionCache() const {
    size_t n = nodes_.size();
    if (congestionCache_.size() != n) congestionCache_.resize(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        if (nodes_[i]) congestionCache_[i] = nodes_[i]->getCongestionFactor();
    }
    congestionCacheFrame_ = currentFrame_;
}

std::vector<int> SubwayGraph::shortestDistancePath(int startIdx, int endIdx) const {
    if (startIdx < 0 || endIdx < 0 || startIdx >= nodes_.size() || endIdx >= nodes_.size()) return {};
    const double INF = 1e18;
    resetPathBuffers(INF);
    pathDist_[startIdx] = 0.0;
    pqContainer_.clear();
    pqContainer_.emplace_back(0.0, startIdx);
    while (!pqContainer_.empty()) {
        std::pop_heap(pqContainer_.begin(), pqContainer_.end(), std::greater<std::pair<double,int>>());
        auto curr = pqContainer_.back();
        double d = curr.first; int u = curr.second;
        pqContainer_.pop_back();
        if (pathVisited_[u]) continue;
        pathVisited_[u] = true;
        if (u == endIdx) break;
        if (u >= adjList_.size()) continue;
        for (const auto& edge : adjList_[u]) {
            int v = edge.getToIndex();
            if (v < 0 || v >= static_cast<int>(nodes_.size())) continue;
            double weight = edge.getLength();
            if (pathDist_[u] + weight < pathDist_[v]) {
                pathDist_[v] = pathDist_[u] + weight;
                pathPrev_[v] = u;
                pqContainer_.emplace_back(pathDist_[v], v);
                std::push_heap(pqContainer_.begin(), pqContainer_.end(), std::greater<std::pair<double,int>>());
            }
        }
    }
    std::vector<int> path;
    for (int at = endIdx; at != -1; at = pathPrev_[at]) path.push_back(at);
    std::reverse(path.begin(), path.end());
    return (path.front() == startIdx) ? path : std::vector<int>();
}

std::vector<int> SubwayGraph::shortestTimePath(int startIdx, int endIdx) const {
    if (startIdx < 0 || endIdx < 0 || startIdx >= nodes_.size() || endIdx >= nodes_.size()) return {};
    const double INF = 1e18;
    resetPathBuffers(INF);
    pathDist_[startIdx] = 0.0;
    pqContainer_.clear();
    pqContainer_.emplace_back(0.0, startIdx);
    while (!pqContainer_.empty()) {
        std::pop_heap(pqContainer_.begin(), pqContainer_.end(), std::greater<std::pair<double,int>>());
        auto curr = pqContainer_.back();
        double d = curr.first; int u = curr.second;
        pqContainer_.pop_back();
        if (pathVisited_[u]) continue;
        pathVisited_[u] = true;
        if (u == endIdx) break;
        if (u >= adjList_.size()) continue;
        for (const auto& edge : adjList_[u]) {
            int v = edge.getToIndex();
            if (v < 0 || v >= static_cast<int>(nodes_.size())) continue;
            if (!nodes_[v]) continue;
            double edgeTime = edge.getPassThroughTime();
            double edgeCongestionPenalty = edge.getCongestionLevel() * 5.0;
            double nodeTime = nodes_[v] ? nodes_[v]->getPassThroughTime() : 1.0;
            double nodeCongestionPenalty = nodes_[v] ? nodes_[v]->getCongestionFactor() * 3.0 : 0.0;
            double weight = edgeTime + edgeCongestionPenalty + nodeTime + nodeCongestionPenalty;
            if (pathDist_[u] + weight < pathDist_[v]) {
                pathDist_[v] = pathDist_[u] + weight;
                pathPrev_[v] = u;
                pqContainer_.emplace_back(pathDist_[v], v);
                std::push_heap(pqContainer_.begin(), pqContainer_.end(), std::greater<std::pair<double,int>>());
            }
        }
    }
    std::vector<int> path;
    for (int at = endIdx; at != -1; at = pathPrev_[at]) path.push_back(at);
    std::reverse(path.begin(), path.end());
    return (path.front() == startIdx) ? path : std::vector<int>();
}

std::vector<int> SubwayGraph::multiObjectivePath(int startIdx, int endIdx) const {
    if (startIdx < 0 || endIdx < 0 || startIdx >= nodes_.size() || endIdx >= nodes_.size()) return {};
    const double INF = 1e18;
    resetPathBuffers(INF);
    pathDist_[startIdx] = 0.0;
    if (congestionCacheFrame_ != currentFrame_) rebuildCongestionCache();
    pqContainer_.clear();
    pqContainer_.emplace_back(0.0, startIdx);
    while (!pqContainer_.empty()) {
        std::pop_heap(pqContainer_.begin(), pqContainer_.end(), std::greater<std::pair<double,int>>());
        auto curr = pqContainer_.back();
        double d = curr.first; int u = curr.second;
        pqContainer_.pop_back();
        if (pathVisited_[u]) continue;
        pathVisited_[u] = true;
        if (u == endIdx) break;
        if (u >= adjList_.size()) continue;
        for (const auto& edge : adjList_[u]) {
            int v = edge.getToIndex();
            if (v < 0 || v >= static_cast<int>(nodes_.size())) continue;
            if (v >= nodes_.size() || !nodes_[v]) continue;
            double distanceWeight = edge.getLength();
            double timeWeight = edge.getPassThroughTime() + (nodes_[v] ? nodes_[v]->getPassThroughTime() : 1.0);
            double currentCongestionWeight = congestionCache_[v];
            double futureCongestionEstimate = estimateFutureCongestion(v, endIdx);
            double congestionWeight = 0.7 * currentCongestionWeight + 0.3 * futureCongestionEstimate;
            double transitionPenalty = 0.0;
            if (nodes_[u] && nodes_[v]) {
                std::string fromCode = nodes_[u]->getTypeCode();
                std::string toCode = nodes_[v]->getTypeCode();
                if ((fromCode == "SECURITY" && toCode == "HALL") ||
                    (fromCode == "TICKET" && toCode == "HALL") ||
                    (fromCode == "GATE" && toCode == "HALL") ||
                    (fromCode == "GATE" && toCode == "SECURITY")) {
                    transitionPenalty = 50.0;
                }
            }
            double weight = 0.3 * distanceWeight + 0.4 * timeWeight + 0.3 * congestionWeight + transitionPenalty;
            if (pathDist_[u] + weight < pathDist_[v]) {
                pathDist_[v] = pathDist_[u] + weight;
                pathPrev_[v] = u;
                pqContainer_.emplace_back(pathDist_[v], v);
                std::push_heap(pqContainer_.begin(), pqContainer_.end(), std::greater<std::pair<double,int>>());
            }
        }
    }
    std::vector<int> path;
    for (int at = endIdx; at != -1; at = pathPrev_[at]) path.push_back(at);
    std::reverse(path.begin(), path.end());
    return (path.front() == startIdx) ? path : std::vector<int>();
}

double SubwayGraph::estimateFutureCongestion(int startIdx, int endIdx, int lookAheadSteps) const {
    if (startIdx < 0 || endIdx < 0 || startIdx >= nodes_.size()) return 0.0;
    ensurePathBuffers();
    size_t n = nodes_.size();
    double totalCongestion = 0.0;
    int steps = 0;
    std::fill(estVisited_.begin(), estVisited_.end(), false);
    estVisited_[startIdx] = true;
    while (!estQueue_.empty()) estQueue_.pop();
    estQueue_.push(startIdx);
    std::fill(estLevel_.begin(), estLevel_.end(), -1);
    estLevel_[startIdx] = 0;
    while (!estQueue_.empty() && steps < lookAheadSteps) {
        int u = estQueue_.front(); estQueue_.pop();
        if (estLevel_[u] > lookAheadSteps) break;
        if (u != startIdx) {
            if (u < n && nodes_[u]) { totalCongestion += congestionCache_[u]; steps++; }
        }
        if (u >= adjList_.size()) continue;
        for (const auto& edge : adjList_[u]) {
            int v = edge.getToIndex();
            if (v >= 0 && v < n && !estVisited_[v] && estLevel_[v] == -1) {
                estVisited_[v] = true;
                estLevel_[v] = estLevel_[u] + 1;
                estQueue_.push(v);
            }
        }
    }
    return steps > 0 ? totalCongestion / steps : 0.0;
}

int SubwayGraph::addNode(std::unique_ptr<AbstractNode> node) {
    if (!node) return -1;
    std::string id = node->getId();
    if (idToIndex_.count(id)) { std::cout << "\u8282\u70b9\u91cd\u590d: ID " << id << " \u5df2\u5b58\u5728" << std::endl; return -1; }
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

bool SubwayGraph::removeNode(const std::string& id) {
    int idx = getIndex(id);
    if (idx < 0 || idx >= nodes_.size()) return false;
    for (auto& edges : adjList_) {
        edges.erase(std::remove_if(edges.begin(), edges.end(), [idx](const Edge& e) { return e.getToIndex() == idx; }), edges.end());
    }
    adjList_[idx].clear();
    int lastIdx = static_cast<int>(nodes_.size()) - 1;
    if (idx != lastIdx) {
        std::string movedId = indexToId_[lastIdx];
        idToIndex_[movedId] = idx;
        indexToId_[idx] = movedId;
        nodes_[idx] = std::move(nodes_[lastIdx]);
        adjList_[idx] = std::move(adjList_[lastIdx]);
        for (auto& edges : adjList_) {
            for (auto& edge : edges) { if (edge.getToIndex() == lastIdx) edge.setToIndex(idx); }
        }
        std::cout << "\u8b66\u544a: \u5220\u9664\u8282\u70b9 " << id << " (\u7d22\u5f15" << idx << "), \u539f\u672b\u5c3e\u8282\u70b9 " << movedId << " \u5df2\u79fb\u52a8\u5230\u7d22\u5f15 " << idx << std::endl;
    }
    nodes_.pop_back();
    adjList_.pop_back();
    idToIndex_.erase(id);
    indexToId_.pop_back();
    return true;
}

AbstractNode* SubwayGraph::getNode(const std::string& id) const {
    int idx = getIndex(id);
    return (idx >= 0) ? nodes_[idx].get() : nullptr;
}

AbstractNode* SubwayGraph::getNode(int index) const {
    return (index >= 0 && index < nodes_.size()) ? nodes_[index].get() : nullptr;
}

const std::vector<std::unique_ptr<AbstractNode>>& SubwayGraph::getAllNodes() const { return nodes_; }

bool SubwayGraph::addEdge(const std::string& fromId, const std::string& toId, const Edge& edge) {
    int fromIdx = getIndex(fromId);
    int toIdx = getIndex(toId);
    if (fromIdx < 0 || toIdx < 0) return false;
    Edge newEdge = edge;
    newEdge.setToIndex(toIdx);
    adjList_[fromIdx].push_back(newEdge);
    return true;
}

bool SubwayGraph::removeEdge(const std::string& fromId, const std::string& toId) {
    int fromIdx = getIndex(fromId);
    int toIdx = getIndex(toId);
    if (fromIdx < 0 || toIdx < 0) return false;
    auto& edges = adjList_[fromIdx];
    edges.erase(std::remove_if(edges.begin(), edges.end(), [toIdx](const Edge& e) { return e.getToIndex() == toIdx; }), edges.end());
    return true;
}

const Edge* SubwayGraph::getEdge(const std::string& fromId, const std::string& toId) const { return getEdge(getIndex(fromId), getIndex(toId)); }

const Edge* SubwayGraph::getEdge(int fromIdx, int toIdx) const {
    if (fromIdx < 0 || fromIdx >= adjList_.size()) return nullptr;
    for (const auto& e : adjList_[fromIdx]) { if (e.getToIndex() == toIdx) return &e; }
    return nullptr;
}

Edge* SubwayGraph::getEdgeMutable(int fromIdx, int toIdx) const {
    if (fromIdx < 0 || fromIdx >= adjList_.size()) return nullptr;
    for (auto& e : adjList_[fromIdx]) { if (e.getToIndex() == toIdx) return &e; }
    return nullptr;
}

Edge* SubwayGraph::getEdgeMutable(const std::string& fromId, const std::string& toId) const { return getEdgeMutable(getIndex(fromId), getIndex(toId)); }

const std::vector<Edge>& SubwayGraph::getNeighbors(int index) const {
    static const std::vector<Edge> empty;
    return (index >= 0 && index < adjList_.size()) ? adjList_[index] : empty;
}

int SubwayGraph::getIndex(const std::string& id) const {
    auto it = idToIndex_.find(id);
    return (it != idToIndex_.end()) ? it->second : -1;
}

const std::string& SubwayGraph::getId(int index) const {
    static const std::string empty;
    return (index >= 0 && index < indexToId_.size()) ? indexToId_[index] : empty;
}

bool SubwayGraph::hasNode(const std::string& id) const { return idToIndex_.count(id) > 0; }

std::vector<int> SubwayGraph::findPath(const std::string& startId, const std::string& endId, PathStrategy strategy) const {
    int startIdx = getIndex(startId);
    int endIdx = getIndex(endId);
    if (startIdx < 0 || endIdx < 0) { std::cout << "\u8def\u5f84\u89c4\u5212\u9519\u8bef: \u8d77\u70b9\u6216\u7ec8\u70b9\u4e0d\u5b58\u5728 - " << startId << " -> " << endId << std::endl; return {}; }
    if (startIdx >= nodes_.size() || endIdx >= nodes_.size()) { std::cout << "\u8def\u5f84\u89c4\u5212\u9519\u8bef: \u7d22\u5f15\u8d85\u51fa\u8303\u56f4" << std::endl; return {}; }
    switch (strategy) {
    case PathStrategy::SHORTEST_DISTANCE: return shortestDistancePath(startIdx, endIdx);
    case PathStrategy::SHORTEST_TIME: return shortestTimePath(startIdx, endIdx);
    case PathStrategy::MULTI_OBJECTIVE_OPTIMIZATION: return multiObjectivePath(startIdx, endIdx);
    default: return shortestTimePath(startIdx, endIdx);
    }
}

bool SubwayGraph::canTraverseEdge(int fromIdx, int toIdx) const {
    const Edge* e = getEdge(fromIdx, toIdx);
    if (!e) return false;
    AbstractNode* next = getNode(toIdx);
    return e->canEnter() && (!next || next->canEnter());
}

bool SubwayGraph::saveToCSV(const std::string& filePath) const {
    std::ofstream file(filePath);
    if (!file.is_open()) {
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
    for (const auto& node : nodes_) {
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
        if (nodeType == "SECURITY") {
            file << props["scannerCount"] << "," << props["checkTimePerPerson"] << "," << props["hasBannedItem"] << ",";
            file << ",,,"; // ticket
            file << ",,";  // gate
            file << ",,,,"; // exit
            file << ",,,,,"; // platform
            file << "\n";   // stair
        }
        else if (nodeType == "TICKET") {
            file << ",,,"; // sec
            file << props["windowCount"] << "," << props["buyTimePerPerson"] << "," << props["hasAutoMachine"] << ",";
            file << ",,";  // gate
            file << ",,,,"; // exit
            file << ",,,,,"; // platform
            file << "\n";   // stair
        }
        else if (nodeType == "GATE") {
            file << ",,,"; // sec
            file << ",,,"; // ticket
            file << props["gateCount"] << "," << props["isBidirectional"] << ",";
            file << ",,,,"; // exit
            file << ",,,,,"; // platform
            file << "\n";   // stair
        }
        else if (nodeType == "EXIT") {
            file << ",,,"; // sec
            file << ",,,"; // ticket
            file << ",,";  // gate
            file << props["exitName"] << "," << props["connectedStreet"] << "," << props["isOneWay"] << "," << props["totalExits"] << ",";
            file << ",,,,,"; // platform
            file << "\n";   // stair
        }
        else if (nodeType == "PLATFORM") {
            file << ",,,"; // sec
            file << ",,,"; // ticket
            file << ",,";  // gate
            file << ",,,,"; // exit
            file << props["lineName"] << "," << props["direction"] << "," << props["waitCap"] << "," << props["hasScreenDoor"] << "," << props["nextTrainIn"] << ",";
            file << "\n";   // stair
        }
        else if (nodeType == "STAIR") {
            file << ",,,"; // sec
            file << ",,,"; // ticket
            file << ",,";  // gate
            file << ",,,,"; // exit
            file << ",,,,,"; // platform
            file << props["stepCount"] << "\n";
        }
        else {
            file << ",,,,,,," << ",,,,,,," << ",,\n"; // 其余填空补齐
        }
    }

    // 批量写入边 (Edges)
    for (size_t u = 0; u < adjList_.size(); ++u) {
        for (const auto& edge : adjList_[u]) {
            int v = edge.getToIndex();
            if (v < 0 || v >= static_cast<int>(nodes_.size())) continue;

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
bool SubwayGraph::loadFromCSV(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
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

    while (std::getline(file, line)) {
        lineNumber++;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;

        try {
            std::vector<std::string> values;
            std::string field;
            bool inQuotes = false;
            for (char c : line) {
                if (c == '"') { inQuotes = !inQuotes; }
                else if (c == ',' && !inQuotes) { values.push_back(field); field.clear(); }
                else { field += c; }
            }
            values.push_back(field);

            std::string recordType = values.size() > 0 ? values[0] : "";
            if (recordType == "NODE" && values.size() >= 12) {
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

                if (typeCode == "SECURITY" && values.size() > 18) {
                    props["scannerCount"] = values[16];
                    props["checkTimePerPerson"] = values[17];
                    props["hasBannedItem"] = values[18];
                }
                else if (typeCode == "TICKET" && values.size() > 21) {
                    props["windowCount"] = values[19];
                    props["buyTimePerPerson"] = values[20];
                    props["hasAutoMachine"] = values[21];
                }
                else if (typeCode == "GATE" && values.size() > 23) {
                    props["gateCount"] = values[22];
                    props["isBidirectional"] = values[23];
                }
                else if (typeCode == "EXIT" && values.size() > 27) {
                    props["exitName"] = values[24];
                    props["connectedStreet"] = values[25];
                    props["isOneWay"] = values[26];
                    props["totalExits"] = values[27];
                }
                else if (typeCode == "PLATFORM" && values.size() > 32) {
                    props["lineName"] = values[28];
                    props["direction"] = values[29];
                    props["waitCap"] = values[30];
                    props["hasScreenDoor"] = values[31];
                    props["nextTrainIn"] = values[32];
                }
                else if (typeCode == "STAIR" && values.size() > 33) {
                    props["stepCount"] = values[33];
                }

                nodePropsList.push_back(props);
                nodeTypeCodes.push_back(typeCode);
            }
            else if (recordType == "EDGE" && values.size() >= 15) {
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
        catch (...) {
            errorCount++;
        }
    }

    // 批量添加节点
    for (size_t i = 0; i < nodePropsList.size(); ++i) {
        auto node = NodeFactory::createNode(nodeTypeCodes[i], nodePropsList[i]);
        if (node) addNode(std::move(node));
    }

    // 批量添加边
    for (size_t i = 0; i < edgeFromIds.size(); ++i) {
        addEdge(edgeFromIds[i], edgeToIds[i], edgeList[i]);
    }

    std::cout << "CSV加载完成 - 成功记录数: " << successCount << " / 错误忽略: " << errorCount << std::endl;
    return true;
}

void SubwayGraph::visualize() const {
    std::cout << "\n=== \u5730\u94c1\u7ad9\u62d3\u6251\u7ed3\u6784 ===" << std::endl;
    for (size_t i = 0; i < nodes_.size(); ++i) {
        std::cout << "\u8282\u70b9 " << i << " (" << nodes_[i]->getTypeName() << "): " << nodes_[i]->getId() << " - \u697c\u5c42 " << nodes_[i]->getFloor() << ", \u4f4d\u7f6e(" << nodes_[i]->getPos().x << "," << nodes_[i]->getPos().y << ")" << ", \u8d1f\u8f7d: " << nodes_[i]->getCurrentLoad() << "/" << nodes_[i]->getCapacity() << ", \u62e5\u5835: " << nodes_[i]->getCongestionFactor() << ", \u961f\u5217: " << nodes_[i]->getQueueLength() << std::endl;
        for (const auto& edge : adjList_[i]) {
            int to = edge.getToIndex();
            if (to >= 0 && to < static_cast<int>(nodes_.size())) std::cout << "  -> \u8fde\u63a5\u5230\u8282\u70b9 " << to << " (" << nodes_[to]->getTypeName() << ") \u957f\u5ea6: " << edge.getLength() << ", \u6276\u68af: " << (edge.getIsEscalator() ? "\u662f" : "\u5426") << std::endl;
        }
    }
    std::cout << "======================" << std::endl;
}

void SubwayGraph::update(double deltaTime) { ++currentFrame_; }

int SubwayGraph::getEdgeCount() const {
    int count = 0;
    for (const auto& edges : adjList_) count += static_cast<int>(edges.size());
    return count;
}

void SubwayGraph::updateAllEdgeCongestion() const {
    for (const auto& edges : adjList_) {
        for (const auto& edge : edges) {
            edge.updateCongestion();
        }
    }
}
