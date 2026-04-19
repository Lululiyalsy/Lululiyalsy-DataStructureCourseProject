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
    if (startIdx < 0 || endIdx < 0 || startIdx >= static_cast<int>(nodes_.size()) || endIdx >= static_cast<int>(nodes_.size())) return {};
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
        if (u >= static_cast<int>(adjList_.size())) continue;
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
    if (startIdx < 0 || endIdx < 0 || startIdx >= static_cast<int>(nodes_.size()) || endIdx >= static_cast<int>(nodes_.size())) return {};
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
        if (u >= static_cast<int>(adjList_.size())) continue;
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
    if (startIdx < 0 || endIdx < 0 || startIdx >= static_cast<int>(nodes_.size()) || endIdx >= static_cast<int>(nodes_.size())) return {};
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
        if (u >= static_cast<int>(adjList_.size())) continue;
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
    if (startIdx < 0 || endIdx < 0 || startIdx >= static_cast<int>(nodes_.size())) return 0.0;
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
            if (u < static_cast<int>(n) && nodes_[u]) { totalCongestion += congestionCache_[u]; steps++; }
        }
        if (u >= static_cast<int>(adjList_.size())) continue;
        for (const auto& edge : adjList_[u]) {
            int v = edge.getToIndex();
            if (v >= 0 && v < static_cast<int>(n) && !estVisited_[v] && estLevel_[v] == -1) {
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
    if (idx < 0 || idx >= static_cast<int>(nodes_.size())) return false;
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
    return (index >= 0 && index < static_cast<int>(nodes_.size())) ? nodes_[index].get() : nullptr;
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
    if (fromIdx < 0 || fromIdx >= static_cast<int>(adjList_.size())) return nullptr;
    for (const auto& e : adjList_[fromIdx]) { if (e.getToIndex() == toIdx) return &e; }
    return nullptr;
}

Edge* SubwayGraph::getEdgeMutable(int fromIdx, int toIdx) const {
    if (fromIdx < 0 || fromIdx >= static_cast<int>(adjList_.size())) return nullptr;
    for (auto& e : adjList_[fromIdx]) { if (e.getToIndex() == toIdx) return &e; }
    return nullptr;
}

Edge* SubwayGraph::getEdgeMutable(const std::string& fromId, const std::string& toId) const { return getEdgeMutable(getIndex(fromId), getIndex(toId)); }

const std::vector<Edge>& SubwayGraph::getNeighbors(int index) const {
    static const std::vector<Edge> empty;
    return (index >= 0 && index < static_cast<int>(adjList_.size())) ? adjList_[index] : empty;
}

int SubwayGraph::getIndex(const std::string& id) const {
    auto it = idToIndex_.find(id);
    return (it != idToIndex_.end()) ? it->second : -1;
}

const std::string& SubwayGraph::getId(int index) const {
    static const std::string empty;
    return (index >= 0 && index < static_cast<int>(indexToId_.size())) ? indexToId_[index] : empty;
}

bool SubwayGraph::hasNode(const std::string& id) const { return idToIndex_.count(id) > 0; }

std::vector<int> SubwayGraph::findPath(const std::string& startId, const std::string& endId, PathStrategy strategy) const {
    int startIdx = getIndex(startId);
    int endIdx = getIndex(endId);
    if (startIdx < 0 || endIdx < 0) { std::cout << "\u8def\u5f84\u89c4\u5212\u9519\u8bef: \u8d77\u70b9\u6216\u7ec8\u70b9\u4e0d\u5b58\u5728 - " << startId << " -> " << endId << std::endl; return {}; }
    if (startIdx >= static_cast<int>(nodes_.size()) || endIdx >= static_cast<int>(nodes_.size())) { std::cout << "\u8def\u5f84\u89c4\u5212\u9519\u8bef: \u7d22\u5f15\u8d85\u51fa\u8303\u56f4" << std::endl; return {}; }
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

bool SubwayGraph::loadFromCSV(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) { std::cout << "\u65e0\u6cd5\u6253\u5f00\u6587\u4ef6: " << filePath << std::endl; return false; }
    std::string header;
    std::getline(file, header);
    if (header.find("recordType") == std::string::npos || header.find("type") == std::string::npos || header.find("id") == std::string::npos) {
        std::cout << "CSV\u6587\u4ef6\u683c\u5f0f\u9519\u8bef: \u7f3a\u5c11\u5fc5\u8981\u5b57\u6bb5" << std::endl; return false;
    }
    std::string line;
    int lineNumber = 1; int successCount = 0; int errorCount = 0;
    std::vector<std::string> errors;
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
            for (size_t i = 0; i < line.length(); ++i) {
                char c = line[i];
                if (c == '"') inQuotes = !inQuotes;
                else if (c == ',' && !inQuotes) { values.push_back(field); field.clear(); }
                else field += c;
            }
            values.push_back(field);
            if (values.size() < 2) { errors.push_back("\u7b2c " + std::to_string(lineNumber) + " \u884c: \u5b57\u6bb5\u6570\u91cf\u4e0d\u8db3"); errorCount++; continue; }
            std::string recordType = values[0];
            if (recordType == "NODE" && values.size() >= 7) {
                std::map<std::string, std::string> props;
                props["type"] = values[1]; props["id"] = values[2]; props["floor"] = values[3];
                props["x"] = values[4]; props["y"] = values[5]; props["capacity"] = values[6];
                props["baseVelocity"] = (values.size() > 7 && !values[7].empty()) ? values[7] : "1.0";
                props["sensitivity"] = (values.size() > 8 && !values[8].empty()) ? values[8] : "1.0";
                std::string typeCode = props["type"];
                size_t idx = 9;
                if (typeCode == "SECURITY" && values.size() > idx + 2) { props["scannerCount"] = values[idx++]; props["checkTimePerPerson"] = values[idx++]; props["hasBannedItem"] = values[idx++]; }
                else if (typeCode == "TICKET" && values.size() > idx + 2) { props["windowCount"] = values[idx++]; props["buyTimePerPerson"] = values[idx++]; props["hasAutoMachine"] = values[idx++]; }
                else if (typeCode == "GATE" && values.size() > idx + 1) { props["gateCount"] = values[idx++]; props["isBidirectional"] = values[idx++]; }
                else if (typeCode == "EXIT" && values.size() > idx + 3) { props["exitName"] = values[idx++]; props["connectedStreet"] = values[idx++]; props["isOneWay"] = values[idx++]; props["totalExits"] = values[idx++]; }
                else if (typeCode == "PLATFORM" && values.size() > idx + 4) { props["lineName"] = values[idx++]; props["direction"] = values[idx++]; props["waitCap"] = values[idx++]; props["hasScreenDoor"] = values[idx++]; props["nextTrainIn"] = values[idx++]; }
                else if (typeCode == "STAIR" && values.size() > idx + 1) { props["stepCount"] = values[idx++]; props["direction"] = values[idx++]; }
                nodePropsList.push_back(props); nodeTypeCodes.push_back(typeCode);
            } else if (recordType == "EDGE" && values.size() >= 6) {
                Edge e; e.setLength(safeStod(values[3], 10.0)); e.setWidth(safeStod(values[4], 2.0));
                if (values.size() > 6) e.setBaseVelocity(safeStod(values[6], 1.0));
                if (values.size() > 7) e.setIsEscalator(safeStoi(values[7], 0) == 1);
                edgeFromIds.push_back(values[1]); edgeToIds.push_back(values[2]); edgeList.push_back(e);
            } else { errors.push_back("\u7b2c " + std::to_string(lineNumber) + " \u884c: \u8bb0\u5f55\u7c7b\u578b\u6216\u5b57\u6bb5\u6570\u91cf\u9519\u8bef"); errorCount++; continue; }
            successCount++;
        } catch (const std::exception& e) { errorCount++; errors.push_back("\u7b2c " + std::to_string(lineNumber) + " \u884c\u5f02\u5e38: " + e.what()); }
        if (errorCount > 0 && successCount > 0 && static_cast<double>(errorCount) / (successCount + errorCount) > 0.5) { std::cout << "\u9519\u8bef\u7387\u8fc7\u9ad8\uff0c\u505c\u6b62\u52a0\u8f7d" << std::endl; break; }
    }
    for (size_t i = 0; i < nodePropsList.size(); ++i) {
        auto& props = nodePropsList[i]; auto& typeCode = nodeTypeCodes[i];
        auto node = NodeFactory::createNode(typeCode, props);
        if (node) addNode(std::move(node)); else { errorCount++; errors.push_back("\u8282\u70b9\u521b\u5efa\u5931\u8d25: ID=" + props["id"]); }
    }
    for (size_t i = 0; i < edgeFromIds.size(); ++i) {
        if (!addEdge(edgeFromIds[i], edgeToIds[i], edgeList[i])) { errorCount++; errors.push_back("\u8fb9\u6dfb\u52a0\u5931\u8d25: " + edgeFromIds[i] + " -> " + edgeToIds[i]); }
    }
    std::cout << "CSV\u52a0\u8f7d\u5b8c\u6210 - \u6210\u529f: " << successCount << ", \u5931\u8d25: " << errorCount << std::endl;
    if (!errors.empty()) { std::cout << "\u9519\u8bef\u8be6\u60c5:" << std::endl; for (const auto& error : errors) std::cout << "  " << error << std::endl; }
    return errorCount == 0 && successCount > 0;
}

bool SubwayGraph::saveToCSV(const std::string& filePath) const {
    std::ofstream file(filePath);
    if (!file.is_open()) { std::cout << "\u65e0\u6cd5\u521b\u5efa\u6587\u4ef6: " << filePath << std::endl; return false; }
    file << "recordType,type,id,floor,x,y,capacity,baseVelocity,sensitivity,"
        << "length,width,scannerCount,checkTimePerPerson,hasBannedItem,"
        << "windowCount,buyTimePerPerson,hasAutoMachine,gateCount,isBidirectional,"
        << "exitName,connectedStreet,isOneWay,totalExits,lineName,direction,"
        << "waitCap,hasScreenDoor,nextTrainIn,stepCount,isEscalator\n";
    for (const auto& node : nodes_) {
        std::map<std::string, std::string> props = node->toProperties();
        file << "NODE," << props["type"] << "," << props["id"] << ","
            << props["floor"] << "," << props["x"] << "," << props["y"] << ","
            << props["capacity"] << "," << props["baseVelocity"] << "," << props["sensitivity"];
        std::string nodeType = props["type"];
        if (nodeType == "SECURITY") file << ",,,,," << props["scannerCount"] << "," << props["checkTimePerPerson"] << "," << props["hasBannedItem"] << ",,,,,,,,,,,,,,,\n";
        else if (nodeType == "TICKET") file << ",,,,,,,," << props["windowCount"] << "," << props["buyTimePerPerson"] << "," << props["hasAutoMachine"] << ",,,,,,,,,,,,,\n";
        else if (nodeType == "GATE") file << ",,,,,,,,,," << props["gateCount"] << "," << props["isBidirectional"] << ",,,,,,,,,,,\n";
        else if (nodeType == "EXIT") file << ",,,,,,,,,,,," << props["exitName"] << "," << props["connectedStreet"] << "," << props["isOneWay"] << "," << props["totalExits"] << ",,,,,,,\n";
        else if (nodeType == "PLATFORM") file << ",,,,,,,,,,,,,,,," << props["lineName"] << "," << props["direction"] << "," << props["waitCap"] << "," << props["hasScreenDoor"] << "," << props["nextTrainIn"] << ",,\n";
        else if (nodeType == "STAIR") file << ",,,,,,,,,,,,,,,," << props["stepCount"] << "," << props["direction"] << ",,\n";
        else file << ",,,,,,,,,,,,,,,,,,,,,,\n";
    }
    for (size_t u = 0; u < adjList_.size(); ++u) {
        for (const auto& edge : adjList_[u]) {
            int v = edge.getToIndex();
            if (v < 0 || v >= static_cast<int>(nodes_.size())) continue;
            file << "EDGE," << indexToId_[u] << "," << indexToId_[v] << "," << edge.getLength() << "," << edge.getWidth() << "," << edge.getBaseVelocity() << "," << (edge.getIsEscalator() ? "1" : "0") << "\n";
        }
    }
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

void SubwayGraph::createDefaultStation() {
    auto hall = std::make_unique<HallNode>("H1", 1, POINT{50, 50}, 200, 1.2, 1.0);
    addNode(std::move(hall));
    auto security = std::make_unique<SecurityNode>("S1", 1, POINT{100, 50}, 50, 1.0, 1.0, 3, 5.0, false);
    addNode(std::move(security));
    auto ticket = std::make_unique<TicketNode>("T1", 1, POINT{100, 100}, 30, 1.0, 1.0, 4, 10.0, true);
    addNode(std::move(ticket));
    auto gate = std::make_unique<GateNode>("G1", 1, POINT{150, 50}, 40, 1.0, 1.0, 6, true);
    addNode(std::move(gate));
    auto platform = std::make_unique<PlatformNode>("P1", 0, POINT{200, 50}, 300, 1.0, 1.0, "Line1", 0, 150, true, 60.0);
    addNode(std::move(platform));
    auto exitNode = std::make_unique<ExitNode>("E1", 1, POINT{0, 50}, 100, 1.5, 1.0, "\u4e3b\u51fa\u53e3", "\u4eba\u6c11\u8def", false, 3);
    addNode(std::move(exitNode));
    auto stair = std::make_unique<StairNode>("ST1", 0, POINT{175, 50}, 30, 0.8, 1.2, 20, 0);
    addNode(std::move(stair));

    Edge e1; e1.setLength(20.0); e1.setWidth(3.0); e1.setBaseVelocity(1.2);
    addEdge("E1", "H1", e1);
    Edge e2; e2.setLength(15.0); e2.setWidth(2.0); e2.setBaseVelocity(1.0);
    addEdge("H1", "T1", e2);
    Edge e3; e3.setLength(15.0); e3.setWidth(2.0); e3.setBaseVelocity(1.0);
    addEdge("H1", "S1", e3);
    Edge e4; e4.setLength(10.0); e4.setWidth(2.0); e4.setBaseVelocity(1.0);
    addEdge("T1", "G1", e4);
    Edge e5; e5.setLength(10.0); e5.setWidth(2.0); e5.setBaseVelocity(1.0);
    addEdge("S1", "G1", e5);
    Edge e6; e6.setLength(25.0); e6.setWidth(2.5); e6.setBaseVelocity(1.0);
    addEdge("G1", "ST1", e6);
    Edge e7; e7.setLength(15.0); e7.setWidth(2.0); e7.setIsEscalator(true); e7.setBaseVelocity(0.5);
    addEdge("ST1", "P1", e7);
    Edge e8; e8.setLength(15.0); e8.setWidth(2.0); e8.setIsEscalator(true); e8.setBaseVelocity(0.5);
    addEdge("P1", "ST1", e8);
    Edge e9; e9.setLength(25.0); e9.setWidth(2.5); e9.setBaseVelocity(1.0);
    addEdge("ST1", "G1", e9);
    Edge e10; e10.setLength(10.0); e10.setWidth(2.0); e10.setBaseVelocity(1.0);
    addEdge("G1", "S1", e10);
    Edge e11; e11.setLength(10.0); e11.setWidth(2.0); e11.setBaseVelocity(1.0);
    addEdge("G1", "T1", e11);
    Edge e12; e12.setLength(15.0); e12.setWidth(2.0); e12.setBaseVelocity(1.0);
    addEdge("S1", "H1", e12);
    Edge e13; e13.setLength(15.0); e13.setWidth(2.0); e13.setBaseVelocity(1.0);
    addEdge("T1", "H1", e13);
    Edge e14; e14.setLength(20.0); e14.setWidth(3.0); e14.setBaseVelocity(1.2);
    addEdge("H1", "E1", e14);

    std::cout << "\u9ed8\u8ba4\u7ad9\u70b9\u5df2\u521b\u5efa: 7\u4e2a\u8282\u70b9, " << getEdgeCount() << "\u6761\u8fb9" << std::endl;
}