#pragma once
#include "Enums.h"
#include "Passenger.h"
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <unordered_map>

class SubwayGraph;

// 客流配置档案
class CrowdProfile {
public:
    std::string name;          // 档案名称（如早高峰/晚高峰）
    double arrival_rate;       // 到达率（人/秒）
    float familiarity_min, familiarity_max; // 熟悉度范围
    float patience_min, patience_max;       // 耐心值范围
    float speed_min, speed_max;             // 速度范围
    float luggage_prob;        // 携带行李概率
    float commute_ratio;       // 通勤比例

    CrowdProfile()
        : arrival_rate(0.5),
        familiarity_min(0.5f), familiarity_max(1.0f),
        patience_min(0.5f), patience_max(1.0f),
        speed_min(0.8f), speed_max(1.2f),
        luggage_prob(0.2f), commute_ratio(0.8f) {}
};

// 时段配置
class TimeSlot {
public:
    int start_second;   // 时段开始秒数
    int end_second;     // 时段结束秒数
    int day_mask;       // 星期掩码（位0-6对应周一-周日）
    CrowdProfile profile; // 对应客流档案

    static int time_to_seconds(int hour, int minute) { return 3600 * hour + minute * 60; } // 时分转秒
    static int weekday_mask() { return (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4); } // 工作日掩码
    static int weekend_mask() { return (1 << 5) | (1 << 6); } // 周末掩码
    static int daily_mask() { return 0b1111111; } // 每天掩码
};

// 虚拟时钟
class VirtualClock {
private:
    double total_sim_seconds;   // 总仿真秒数
    int start_day_of_week;      // 起始星期几 (0=周一)
    int start_hour_of_day;      // 起始小时

public:
    VirtualClock(int start_weekday = 0, int start_hour = 5)
        : total_sim_seconds(start_hour * 3600), start_day_of_week(start_weekday),
        start_hour_of_day(start_hour) {}

    void update(double dt) { total_sim_seconds += dt; }
    int get_current_weekday() const;
    int get_seconds_today() const;
    int get_current_hour() const;
    int get_current_minute() const;
    std::string get_formatted_time() const; // 格式化输出时间
    bool is_weekday() const;  // 是否工作日
    bool is_weekend() const;  // 是否周末
    double get_total_seconds() const { return total_sim_seconds; } // 获取总秒数
};

// 生成统计
struct GenerationStats {
    int weekday_total = 0;     // 工作日生成总数
    int weekend_total = 0;     // 周末生成总数
    int peak_total = 0;        // 高峰期生成总数
    int offpeak_total = 0;     // 非高峰期生成总数
    std::unordered_map<std::string, int> profile_counts; // 各档案生成计数
};

// 乘客生成器
class PassengerGenerator {
private:
    std::vector<TimeSlot> schedule;  // 时段调度表
    VirtualClock& clock;             // 虚拟时钟引用
    mutable std::mt19937 rng;        // 随机数生成器
    int total_generated;             // 总生成数
    GenerationStats stats;           // 生成统计
    SubwayGraph& graphRef;           // 图引用
    std::vector<std::string> platformIds; // 站台ID列表
    int trainPassengerCounter;       // 列车乘客计数器
    int maxOnlinePassengers;         // 最大在线乘客数

public:
    PassengerGenerator(VirtualClock& c, SubwayGraph& graph, int maxOnline = 2500);

    void updateStationLayout();
    std::vector<Passenger> generateTrainPassengers(double dt, int remaining = 9999); // 生成列车下车乘客
    std::string findRandomExit(int floor = 0) const; // 随机选择出口，优先同楼层
    PassengerAttributes generateDefaultAttributes() const; // 生成默认属性
    void initialize_default_schedule(); // 初始化默认调度表
    const CrowdProfile* get_current_profile(); // 获取当前时段客流档案
    std::vector<Passenger> generateEntryPassengers(double dt, int remaining = 9999); // 生成进站乘客
    std::vector<Passenger> generate(double dt, int currentOnlineCount = 0); // 统一生成入口
    void print_stats() const; // 打印生成统计
    int get_total_generated() const { return total_generated; }
};
