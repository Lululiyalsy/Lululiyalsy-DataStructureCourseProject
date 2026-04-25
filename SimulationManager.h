#pragma once
#include "SimEnvironment.h"
#include "Passenger.h"
#include <vector>
#include <string>
#include <unordered_map>

class SubwayGraph;

// 仿真统计
class SimulationStatistics {
public:
    // 节点统计
    struct NodeStats {
        int max_load = 0;          // 最大负载
        int total_visits = 0;      // 总访问次数
        double total_wait_time = 0.0; // 总等待时间
        double avg_congestion = 0.0;  // 平均拥堵度
        int total_queue_time = 0;  // 总排队时间
        int total_queued = 0;      // 总排队次数
        // F4采样字段
        double cumulative_congestion_sum = 0.0;
        long long cumulative_queue_length = 0;
        int sample_count = 0;
    };

    std::unordered_map<std::string, NodeStats> node_statistics; // 各节点统计
    std::vector<double> passenger_times; // 完成乘客通行时间列表
    int total_passengers = 0;            // 总完成乘客数

    void record_node_usage(const std::string& node_id, int load, double congestion, double wait_time = 0.0); // 记录节点使用
    void record_queue_time(const std::string& node_id, int queue_time); // 记录排队时间
    void record_passenger_time(double time); // 记录乘客通行时间
    void print_analysis() const; // 打印统计分析
    void record_tick_for_f4(const SubwayGraph& graph); // F4每秒采样
    void exportF4Data(const std::string& nodeFile, const std::string& passengerFile, const SubwayGraph& graph) const; // F4数据导出
};

// 仿真管理器
class SimulationManager {
private:
    VirtualClock clock;                      // 虚拟时钟
    PassengerGenerator generator;             // 乘客生成器
    std::vector<Passenger> passengers;        // 活跃乘客列表
    std::vector<Passenger> completed_passengers; // 已完成乘客列表
    double sim_time;                          // 仿真时间
    double dt;                                // 时间步长
    SubwayGraph& graph;                       // 地铁图引用
    SimulationStatistics statistics;          // 仿真统计
    int totalTrainPassengersGenerated = 0;    // 列车乘客生成总数
    int totalTrainArrivals = 0;               // 列车到达总数

public:
    SimulationManager(SubwayGraph& g, int maxOnline = 2500); // 构造函数
    void run(int steps); // 运行仿真
};