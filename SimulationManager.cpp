#include "SimulationManager.h"
#include "SubwayGraph.h"
#include "Passenger.h"
#include "Node.h"
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <fstream>

void SimulationStatistics::record_node_usage(const std::string& node_id, int load, double congestion, double wait_time) {
    auto& stats = node_statistics[node_id];
    if (load > stats.max_load) stats.max_load = load;
    stats.total_visits++;
    stats.avg_congestion = (stats.avg_congestion * (stats.total_visits - 1) + congestion) / stats.total_visits;
    stats.total_wait_time += wait_time;
}

void SimulationStatistics::record_queue_time(const std::string& node_id, int queue_time) {
    auto& stats = node_statistics[node_id];
    stats.total_queue_time += queue_time;
    stats.total_queued++;
}

void SimulationStatistics::record_passenger_time(double time) {
    passenger_times.push_back(time);
    total_passengers++;
}

void SimulationStatistics::print_analysis() const {
    std::cout << "\n========== \u4eff\u771f\u7edf\u8ba1\u5206\u6790 ==========" << std::endl;

    // \u62e5\u5835\u6392\u540d
    std::vector<std::pair<std::string, double>> congestion_ranking;
    for (const auto& entry : node_statistics) {
        congestion_ranking.push_back({ entry.first, entry.second.avg_congestion });
    }
    std::sort(congestion_ranking.begin(), congestion_ranking.end(),
        [](const auto& a, const auto& b) { return a.second > b.second; });

    std::cout << "\u62e5\u5835\u6392\u540d\u524d5\u7684\u8282\u70b9:" << std::endl;
    for (int i = 0; i < std::min<int>(5, (int)congestion_ranking.size()); i++) {
        const auto& item = congestion_ranking[i];
        std::cout << "  " << item.first << ": " << std::fixed << std::setprecision(2) << item.second << std::endl;
    }

    // \u5e73\u5747\u901a\u884c\u65f6\u95f4
    if (!passenger_times.empty()) {
        double total_time = 0.0;
        for (double t : passenger_times) total_time += t;
        std::cout << "\u5e73\u5747\u901a\u884c\u65f6\u95f4: " << std::fixed << std::setprecision(2)
            << total_time / passenger_times.size() << " \u79d2" << std::endl;
    }

    // \u6392\u961f\u5206\u6790
    std::cout << "\n\u6392\u961f\u65f6\u95f4\u7edf\u8ba1:" << std::endl;
    std::vector<std::pair<std::string, double>> queue_ranking;
    for (const auto& pair : node_statistics) {
        const std::string& id = pair.first;
        const NodeStats& stats = pair.second;
        if (stats.total_queued > 0) {
            queue_ranking.push_back({ id, static_cast<double>(stats.total_queue_time) / stats.total_queued });
        }
    }
    std::sort(queue_ranking.begin(), queue_ranking.end(),
        [](const auto& a, const auto& b) { return a.second > b.second; });

    for (int i = 0; i < std::min<int>(5, (int)queue_ranking.size()); i++) {
        const auto& item = queue_ranking[i];
        const std::string& id = item.first;
        double avg_queue_time = item.second;
        std::cout << "  " << id << ": " << std::fixed << std::setprecision(2) << avg_queue_time << " \u79d2" << std::endl;
    }

    // \u5cf0\u503c\u8d1f\u8f7d\u5206\u6790
    std::cout << "\u6700\u5927\u8d1f\u8f7d\u8282\u70b9:" << std::endl;
    std::vector<std::pair<std::string, int>> load_ranking;
    for (const auto& entry : node_statistics) {
        load_ranking.push_back({ entry.first, entry.second.max_load });
    }
    std::sort(load_ranking.begin(), load_ranking.end(),
        [](const auto& a, const auto& b) { return a.second > b.second; });

    for (int i = 0; i < std::min<int>(5, (int)load_ranking.size()); i++) {
        const auto& item = load_ranking[i];
        std::cout << "  " << item.first << ": " << item.second << std::endl;
    }

    std::cout << "================================" << std::endl;
}

void SimulationStatistics::record_tick_for_f4(const SubwayGraph& graph) {
    for (const auto& nodePtr : graph.getAllNodes()) {
        if (!nodePtr) continue;
        std::string id = nodePtr->getId();
        auto& stats = node_statistics[id];
        stats.cumulative_congestion_sum += nodePtr->getCongestionFactor();
        stats.cumulative_queue_length += nodePtr->getQueueLength();
        stats.sample_count++;
    }
}

void SimulationStatistics::exportF4Data(const std::string& nodeFile, const std::string& passengerFile, const SubwayGraph& graph) const {
    std::ofstream nFile(nodeFile);
    if (nFile.is_open()) {
        nFile << "node_id,type,floor,x,y,avg_density,avg_queue,total_visits\n";
        for (const auto& pair : node_statistics) {
            const std::string& id = pair.first;
            const NodeStats& stats = pair.second;
            AbstractNode* node = graph.getNode(id);
            if (!node || stats.sample_count == 0) continue;
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
    if (pFile.is_open()) {
        pFile << "passenger_id,travel_time\n";
        for (size_t i = 0; i < passenger_times.size(); ++i) {
            pFile << i << "," << passenger_times[i] << "\n";
        }
        pFile.close();
    }
}

SimulationManager::SimulationManager(SubwayGraph& g, int maxOnline)
    : clock(1, 5), generator(clock, g, maxOnline), sim_time(0.0), dt(1.0), graph(g) {} // 起始时间改为7点

void SimulationManager::stepOnce() {
    for (auto& node : graph.getAllNodes()) {
        node->update(dt);
    }
    graph.update(dt);

    auto new_p = generator.generate(dt, static_cast<int>(passengers.size()));
    passengers.insert(passengers.end(),
        std::make_move_iterator(new_p.begin()),
        std::make_move_iterator(new_p.end()));

    int replansThisFrame = 0;
    const int maxReplansPerFrame = 30;

    for (size_t idx = 0; idx < passengers.size(); ) {
        auto* current_node = graph.getNode(passengers[idx].current_node_id);

        if (!current_node) {
            idx++;
            continue;
        }

        int node_load = current_node->getCurrentLoad();
        int node_capacity = current_node->getCapacity();
        double node_service_time = current_node->getPassThroughTime();

        bool active = passengers[idx].update(dt, node_load, node_capacity, node_service_time, current_node, graph, replansThisFrame, maxReplansPerFrame);

        if (!active) {
            if (passengers[idx].exit_time > 0) {
                statistics.record_passenger_time(passengers[idx].get_travel_time());
            }
            completed_passengers.push_back(std::move(passengers[idx]));
            passengers[idx] = std::move(passengers.back());
            passengers.pop_back();
        } else {
            idx++;
        }
    }
    statistics.record_tick_for_f4(graph);
    if (static_cast<int>(sim_time) % 10 == 0) {
        for (const auto& node : graph.getAllNodes()) {
            statistics.record_node_usage(
                node->getId(),
                node->getCurrentLoad(),
                node->getCongestionFactor()
            );
            if (node->getQueueLength() > 0) {
                statistics.record_queue_time(node->getId(), node->getQueueLength());
            }
        }
    }

    sim_time += dt;
    clock.update(dt);

    // 每秒输出一次状态
    if (static_cast<int>(sim_time) % 1 == 0) {
        std::cout << "[" << clock.get_formatted_time() << "] "
            << "在线人数：" << passengers.size()
            << " | 累计生成：" << generator.get_total_generated()
            << " | 已完成：" << completed_passengers.size()
            << std::endl;

        // 显示乘客状态示例
        if (!passengers.empty()) {
            std::cout << "  乘客状态示例：";
            int count = 0;
            for (const auto& p : passengers) {
                if (count++ >= 5) break;
                std::cout << p.get_state_string() << " ";
            }
            std::cout << std::endl;
        }
    }
}

void SimulationManager::exportFinalReport() {
    // generator.print_stats();
    // statistics.print_analysis();
    statistics.exportF4Data("f4_nodes.csv", "f4_passengers.csv", graph);
}