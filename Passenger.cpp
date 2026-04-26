#include "Passenger.h"
#include "Node.h"
#include "Edge.h"
#include "SubwayGraph.h"
#include <algorithm>
#include <queue>
#include <random>

Passenger::Passenger(int curr_id, int curr_node, int target_node, PassengerAttributes attrs,
    double time, PathStrategy strategy, bool from_train, const SubwayGraph* graph)
    : id(curr_id), current_node_id(curr_node), target_node_id(target_node),
    attributes(attrs), pathStrategy(strategy), spawn_time(time),
    action_timer(0.0), state(from_train ? PassengerState::FROM_TRAIN : PassengerState::SPAWNED),
    exit_time(0.0), queue_start_time(0), queue_position(-1),
    current_path_index(0), current_grid_x(-1), current_grid_y(-1),
    current_edge_from(-1), current_edge_to(-1), transit_timer(0.0), waitTimer(0.0), real_travel_timer(0.0),
    collision_timer(0), isFromTrain(from_train), headingToPlatform(false), lastReplanTime(time), replanInterval(90.0),
    lastCongestionReplanTime(time), graphRef(graph) {}

void Passenger::ensureBfsBuffers(int gw, int gh) const {
    if (bfs_buffer_width != gw || bfs_buffer_height != gh) {
        bfs_visited.assign(gw, std::vector<bool>(gh, false));
        bfs_parent.assign(gw, std::vector<std::pair<int, int>>(gh, { -1, -1 }));
        bfs_buffer_width = gw; bfs_buffer_height = gh;
    } else {
        for (auto& row : bfs_visited) std::fill(row.begin(), row.end(), false);
        for (auto& row : bfs_parent) std::fill(row.begin(), row.end(), std::pair<int, int>(-1, -1));
    }
}

void Passenger::setPath(const std::vector<int>& calculatedPath) {
    path = calculatedPath;
    if (!path.empty()) { current_path_index = 0; target_node_id = (path.size() > 1) ? path[1] : path[0]; }
}

std::pair<int, int> Passenger::getDirectionToTarget(const AbstractNode* node, int target_x, int target_y) const {
    int dx = target_x - current_grid_x;
    int dy = target_y - current_grid_y;
    int move_x = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
    int move_y = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
    return { move_x, move_y };
}

std::vector<std::pair<int, int>> Passenger::findLocalPath(const AbstractNode* node, int target_x, int target_y) const {
    if (current_grid_x < 0 || current_grid_y < 0) return {};
    int gw = node->getGridWidth(); int gh = node->getGridHeight();
    if (current_grid_x >= gw || current_grid_y >= gh) return {};
    ensureBfsBuffers(gw, gh);
    std::queue<std::pair<int, int>> q;
    q.push({ current_grid_x, current_grid_y });
    bfs_visited[current_grid_x][current_grid_y] = true;
    const int dx[] = { 0, 0, 1, -1, 1, -1, 1, -1 };
    const int dy[] = { 1, -1, 0, 0, 1, 1, -1, -1 };
    while (!q.empty()) {
        auto front = q.front(); int x = front.first; int y = front.second; q.pop();
        if (x == target_x && y == target_y && !node->isCellOccupied(x, y)) {
            std::vector<std::pair<int, int>> path;
            std::pair<int, int> current = { x, y };
            while (current.first != -1) { path.push_back(current); current = bfs_parent[current.first][current.second]; }
            std::reverse(path.begin(), path.end());
            return path;
        }
        for (int i = 0; i < 8; ++i) {
            int nx = x + dx[i]; int ny = y + dy[i];
            if (node->isCellValid(nx, ny) && !bfs_visited[nx][ny] && !node->isCellObstacle(nx, ny)) {
                bfs_visited[nx][ny] = true; bfs_parent[nx][ny] = { x, y }; q.push({ nx, ny });
            }
        }
    }
    return {};
}
// [修复] 基于有向图寻路，寻找真正可达的最近出口
std::string Passenger::findNearestExit(const SubwayGraph& graph) const {
    std::string nearestExitId;
    double minDist = 1e18;
    
    for (const auto& node : graph.getAllNodes()) {
        if (node->getTypeCode() == "EXIT") {
            std::string exitId = node->getId();
            // 调用最短距离策略验证是否可达，并获取路径
            std::vector<int> path = graph.findPath(graph.getId(current_node_id), exitId, PathStrategy::SHORTEST_DISTANCE);
            
            if (!path.empty()) {
                // 累加路径上所有边的物理长度
                double dist = 0.0;
                for (size_t i = 0; i < path.size() - 1; ++i) {
                    const Edge* e = graph.getEdge(path[i], path[i+1]);
                    if (e) dist += e->getLength();
                }
                if (dist < minDist) {
                    minDist = dist;
                    nearestExitId = exitId;
                }
            }
        }
    }
    return nearestExitId;
}

AbstractNode* Passenger::getNode(int index) const {
    if (graphRef) return graphRef->getNode(index);
    return nullptr;
}

bool Passenger::update(double dt, int current_node_load, int current_node_capacity,
    double node_service_time, AbstractNode* current_node,
    const SubwayGraph& graph, int& replansThisFrame, int maxReplansPerFrame) {

    if (state == PassengerState::SPAWNED) {
        action_timer += dt;
        AbstractNode* start_node = current_node;
        if (start_node) {
            bool assigned = false;
            if (start_node->occupyCell(1, 1, id)) {
                current_grid_x = 1; current_grid_y = 1; assigned = true;
            }
            if (!assigned) {
                for (int x = 1; x < start_node->getGridWidth() - 1 && !assigned; ++x) {
                    for (int y = 1; y < start_node->getGridHeight() - 1; ++y) {
                        if (start_node->occupyCell(x, y, id)) {
                            current_grid_x = x; current_grid_y = y; assigned = true; break;
                        }
                    }
                }
            }
            if (assigned) {
                state = PassengerState::PATH_FOLLOWING;
            }
            else if (action_timer > 10.0) {
                state = PassengerState::LEFT;
                exit_time = spawn_time + real_travel_timer;
                return false;
            }
        }
        return true;
    }

    if (state == PassengerState::FROM_TRAIN) {
        // 直接转为在站台状态
        state = PassengerState::ON_PLATFORM;
    }

    if (state == PassengerState::ON_PLATFORM) {
        if (current_node && current_node->getTypeCode() == "PLATFORM") {
            std::string exitId = findNearestExit(graph);
            if (!exitId.empty()) {
                std::vector<int> newPath = graph.findPath(
                    graph.getId(current_node_id), exitId, pathStrategy);
                if (!newPath.empty()) {
                    setPath(newPath);
                    state = PassengerState::PATH_FOLLOWING;
                }
            }
        }
    }

    // [修改] 原有逻辑继续执行
    if (state == PassengerState::COLLIDING) {
        if (collision_timer > 2.0) {
            collision_timer = 0;
            state = PassengerState::PATH_FOLLOWING;
        }
        return true;
    }

    if (state == PassengerState::LEFT) return false;

    action_timer += dt;
    collision_timer += dt;
    real_travel_timer += dt; //新增每帧累加真实时间
    bool is_currently_congested = (current_node_capacity > 0) &&
        (current_node_load * 1.0 / current_node_capacity > 0.8);

    // [新增] 检查当前边的拥堵情况
    bool is_edge_congested = false;
    if (current_path_index > 0) {
        std::string prevNodeId = graph.getId(path[current_path_index - 1]);
        std::string currNodeId = graph.getId(current_node_id);
        const Edge* edge = graph.getEdge(prevNodeId, currNodeId);
        if (edge && edge->getCongestionLevel() > 0.8) {
            is_edge_congested = true;
        }
    }

    // [已重构] 原逻辑已移至下方PATH_FOLLOWING宏观转移中，此块已禁用
    if (false && current_node_id == target_node_id) {
        // 到达站台且要乘车，进入候车状态
        if (headingToPlatform && current_node && current_node->getTypeCode() == "PLATFORM") {
            state = PassengerState::WAITING_TRAIN;
            return true;
        }

        if (current_node && (current_node->getTypeCode() == "SECURITY" ||
            current_node->getTypeCode() == "TICKET" ||
            current_node->getTypeCode() == "GATE" ||
            current_node->getTypeCode() == "EXIT")) {

            if (!current_node->joinQueue(id)) {
                if (attributes.patience < 0.3) {
                    if (current_grid_x >= 0 && current_grid_y >= 0) {
                        current_node->releaseCell(current_grid_x, current_grid_y);
                    }
                    advancePath();
                    if (current_path_index >= path.size()) {
                        state = PassengerState::LEFT;
                        exit_time = spawn_time + real_travel_timer;
                    }
                    else {
                        target_node_id = path[current_path_index];
                    }
                    return true;
                }
                state = PassengerState::IN_QUEUE;
                action_timer = 0.0;
                return true;
            }
            else {
                state = PassengerState::IN_QUEUE;
                action_timer = 0.0;
                return true;
            }
        }
        else {
            // 不是服务节点，尝试进入边前往下一节点
            if (!path.empty() && current_path_index < path.size() - 1) {
                int next_node_id = path[current_path_index + 1];
                const Edge* nextEdge = graph.getEdge(current_node_id, next_node_id);
                if (!nextEdge) {
                    state = PassengerState::REPATHING;
                    return true;
                }

                if (!nextEdge->tryEnterEdge()) {
                    state = PassengerState::WAITING_EDGE;
                    return true;
                }

                AbstractNode* nextNode = getNode(next_node_id);
                if (nextNode && !nextNode->canEnter()) {
                    state = PassengerState::WAITING_EDGE;
                    return true;
                }

                // 进入边：释放当前节点网格
                if (current_node && current_grid_x >= 0 && current_grid_y >= 0) {
                    current_node->releaseCell(current_grid_x, current_grid_y);
                }

                // 占用边资源
                Edge* mutableEdge = graph.getEdgeMutable(current_node_id, next_node_id);
                if (mutableEdge) mutableEdge->addOccupant();
                current_edge_from = current_node_id;
                current_edge_to = next_node_id;
                transit_timer = 0.0;
                current_grid_x = -1;
                current_grid_y = -1;
                state = PassengerState::IN_TRANSIT;
                return true;
            }
            else {
                if (current_grid_x >= 0 && current_grid_y >= 0) {
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
    if (state == PassengerState::WAITING_TRAIN) {
        if (current_node && current_node->getTypeCode() == "PLATFORM") {
            PlatformNode* platform = dynamic_cast<PlatformNode*>(current_node);
            if (platform && platform->isTrainArrivingNow()) {
                if (current_grid_x >= 0 && current_grid_y >= 0) {
                    current_node->releaseCell(current_grid_x, current_grid_y);
                }
                state = PassengerState::LEFT;
                exit_time = spawn_time + real_travel_timer;
            }
        }
        return true;
    }

    // 处理IN_TRANSIT状态：在边上移动
    if (state == PassengerState::IN_TRANSIT) {
        const Edge* edge = graph.getEdge(current_edge_from, current_edge_to);
        transit_timer += dt;

        if (edge && transit_timer >= edge->getPassThroughTime()) {
            AbstractNode* nextNode = getNode(current_edge_to);
            if (nextNode && nextNode->canEnter()) {
                // 释放边资源
                Edge* mutableEdge = graph.getEdgeMutable(current_edge_from, current_edge_to);
                if (mutableEdge) mutableEdge->removeOccupant();

                // 到达目标节点
                current_node_id = current_edge_to;
                current_path_index++;
                target_node_id = (current_path_index + 1 < static_cast<int>(path.size()))
                    ? path[current_path_index + 1]
                    : path[current_path_index];

                // 在目标节点分配网格
                bool assigned = false;
                if (nextNode->occupyCell(1, 1, id)) {
                    current_grid_x = 1;
                    current_grid_y = 1;
                    assigned = true;
                }
                if (!assigned) {
                    for (int x = 1; x < nextNode->getGridWidth() - 1 && !assigned; ++x) {
                        for (int y = 1; y < nextNode->getGridHeight() - 1; ++y) {
                            if (nextNode->occupyCell(x, y, id)) {
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

    // 检查是否需要重新规划路径
    if (needsReplanning(current_node, graph) && replansThisFrame < maxReplansPerFrame) {
        state = PassengerState::REPATHING;
        std::string currentId = graph.getId(current_node_id);
        std::string targetId = graph.getId(target_node_id);
        replanPath(currentId, targetId, graph);
        state = PassengerState::PATH_FOLLOWING;
        replansThisFrame++;
    }

    // 只要处于PATH_FOLLOWING状态且路径有效，就尝试转移
    if (state == PassengerState::PATH_FOLLOWING && !path.empty()) {

        // 1. 服务节点必须先排队（注意：EXIT不需要排队，走到出口直接离场）
        if (current_node && (current_node->getTypeCode() == "SECURITY" || current_node->getTypeCode() == "TICKET" || current_node->getTypeCode() == "GATE")) {
            if (current_node->joinQueue(id)) {
                state = PassengerState::IN_QUEUE;
                action_timer = 0.0;
            } else {
                if (attributes.patience < 0.3) {
                    if (current_grid_x >= 0 && current_grid_y >= 0) current_node->releaseCell(current_grid_x, current_grid_y);
                    advancePath();
                    if (current_path_index >= static_cast<int>(path.size())) {
                        state = PassengerState::LEFT;
                        exit_time = spawn_time + real_travel_timer;
                        return false;
                    }
                }
            }
            return true;
        }

        // 2. 出口节点直接离场，绝不排队
        if (current_node && current_node->getTypeCode() == "EXIT") {
            if (current_grid_x >= 0 && current_grid_y >= 0) {
                current_node->releaseCell(current_grid_x, current_grid_y);
                current_grid_x = -1; current_grid_y = -1;
            }
            state = PassengerState::LEFT;
            exit_time = spawn_time + real_travel_timer;
            return false;
        }

        // 3. 站台候车
        if (headingToPlatform && current_node && current_node->getTypeCode() == "PLATFORM") {
            state = PassengerState::WAITING_TRAIN;
            return true;
        }

        // 4. 随时尝试进入下一条边（打破死循环的唯一出路）
        if (current_path_index < static_cast<int>(path.size()) - 1) {
            int next_node_id = path[current_path_index + 1];
            const Edge* nextEdge = graph.getEdge(current_node_id, next_node_id);
            if (!nextEdge) {
                state = PassengerState::REPATHING;
                return true;
            }
            AbstractNode* nextNode = getNode(next_node_id);
            if (!nextEdge->tryEnterEdge() || (nextNode && !nextNode->canEnter())) {
                waitTimer += dt;
                if (waitTimer > 60.0) {
                    if (needsReplanning(current_node, graph)) {
                        std::string currentId = graph.getId(current_node_id);
                        std::string targetId = graph.getId(target_node_id);
                        replanPath(currentId, targetId, graph);
                    }
                    waitTimer = 0.0;
                }
            } else {
                if (current_grid_x >= 0 && current_grid_y >= 0) current_node->releaseCell(current_grid_x, current_grid_y);
                Edge* mutableEdge = graph.getEdgeMutable(current_node_id, next_node_id);
                if (mutableEdge) mutableEdge->addOccupant();
                current_edge_from = current_node_id;
                current_edge_to = next_node_id;
                transit_timer = 0.0;
                current_grid_x = -1;
                current_grid_y = -1;
                state = PassengerState::IN_TRANSIT;
                waitTimer = 0.0;
                return true;
            }
        } else {
            if (current_grid_x >= 0 && current_grid_y >= 0) {
                current_node->releaseCell(current_grid_x, current_grid_y);
                current_grid_x = -1;
                current_grid_y = -1;
            }
            state = PassengerState::LEFT;
            exit_time = spawn_time + real_travel_timer;
            return false;
        }
    }

    // 碰撞策略核心：在节点内部移动
    if (state == PassengerState::PATH_FOLLOWING && current_node && current_grid_x >= 0 && current_grid_y >= 0) {
        collision_timer += dt;

        if (collision_timer >= 0.5) {
            const int dx[] = { 0, 0, 1, -1 };
            const int dy[] = { 1, -1, 0, 0 };

            int dirs[4] = {0, 1, 2, 3};

            static thread_local std::mt19937 local_rng(std::random_device{}());
            std::shuffle(dirs, dirs + 4, local_rng);

            bool moved = false;
            for (int i = 0; i < 4; ++i) {
                int nx = current_grid_x + dx[dirs[i]];
                int ny = current_grid_y + dy[dirs[i]];

                if (!current_node->isCellObstacle(nx, ny) && !current_node->isCellOccupied(nx, ny)) {
                    current_node->moveCell(current_grid_x, current_grid_y, nx, ny, id);
                    current_grid_x = nx;
                    current_grid_y = ny;
                    moved = true;
                    break;
                }
            }

            if (moved) {
                collision_timer = 0.0;
            }
        }
    }


    return true;
}

void Passenger::advancePath() {
    if (current_path_index < static_cast<int>(path.size()) - 1) {
        current_path_index++;
        target_node_id = (current_path_index + 1 < static_cast<int>(path.size())) ? path[current_path_index + 1] : path[current_path_index];
    }
}

bool Passenger::needsReplanning(AbstractNode* currentNode, const SubwayGraph& graph) {
    double now = spawn_time + action_timer;
    bool congestionTriggered = false;
    if (currentNode && currentNode->getCongestionFactor() > 0.8) congestionTriggered = true;
    if (!congestionTriggered && current_path_index < path.size() - 1) {
        for (int i = current_path_index + 1; i < path.size() && i < current_path_index + 3; i++) {
            AbstractNode* futureNode = graph.getNode(path[i]);
            if (futureNode && futureNode->getCongestionFactor() > 0.85) { congestionTriggered = true; break; }
            if (i > 0) {
                const Edge* edge = graph.getEdge(path[i - 1], path[i]);
                if (edge && edge->getCongestionLevel() > 0.8) { congestionTriggered = true; break; }
            }
        }
    }
    if (congestionTriggered && now - lastCongestionReplanTime > congestionReplanCooldown) return true;
    if (now - lastReplanTime > replanInterval) return true;
    if (attributes.speed < 0.5 && action_timer > 30.0) return true;
    return false;
}

void Passenger::replanPath(const std::string& currentId, const std::string& targetId, const SubwayGraph& graph) {
    PathStrategy currentStrategy = pathStrategy;
    bool wasCongestionTriggered = isPathCongested(graph);
    if (wasCongestionTriggered) currentStrategy = PathStrategy::MULTI_OBJECTIVE_OPTIMIZATION;
    std::vector<int> newPath = graph.findPath(currentId, targetId, currentStrategy);
    if (!newPath.empty()) {
        path = newPath; current_path_index = 0; target_node_id = path[0];
        double now = spawn_time + action_timer;
        lastReplanTime = now;
        if (wasCongestionTriggered) lastCongestionReplanTime = now;
    }
}

bool Passenger::isPathCongested(const SubwayGraph& graph) const {
    for (int i = current_path_index; i < path.size() - 1; i++) {
        AbstractNode* node = graph.getNode(path[i]);
        if (node && node->getCongestionFactor() > 0.7) return true;
        if (i > 0) {
            const Edge* edge = graph.getEdge(path[i - 1], path[i]);
            if (edge && edge->getCongestionLevel() > 0.7) return true;
        }
    }
    return false;
}

std::string Passenger::get_state_string() const {
    switch (state) {
    case PassengerState::SPAWNED: return "\u751f\u6210";
    case PassengerState::ENTERING: return "\u8fdb\u7ad9";
    case PassengerState::TICKETING: return "\u8d2d\u7968";
    case PassengerState::SECURITY_CHECK: return "\u5b89\u68c0";
    case PassengerState::MOVING_TO_PLATFORM: return "\u53bb\u7ad9\u53f0";
    case PassengerState::FROM_TRAIN: return "\u5217\u8f66\u4e0b\u8f66";
    case PassengerState::ON_PLATFORM: return "\u7ad9\u53f0\u7b49\u5f85";
    case PassengerState::WAITING_TRAIN: return "\u5019\u8f66";
    case PassengerState::BOARDING: return "\u4e58\u8f66";
    case PassengerState::MOVING_TO_EXIT: return "\u53bb\u51fa\u53e3";
    case PassengerState::EXITING: return "\u51fa\u7ad9";
    case PassengerState::LEFT: return "\u79bb\u5f00";
    case PassengerState::IN_QUEUE: return "\u6392\u961f";
    case PassengerState::PATH_FOLLOWING: return "\u8def\u5f84\u8ddf\u968f";
    case PassengerState::IN_TRANSIT: return "\u901a\u9053\u4e2d";
    case PassengerState::WAITING_EDGE: return "\u7b49\u5f85\u901a\u9053";
    case PassengerState::REPATHING: return "\u91cd\u65b0\u89c4\u5212";
    case PassengerState::COLLIDING: return "\u78b0\u649e\u7b49\u5f85";
    default: return "\u672a\u77e5";
    }
}