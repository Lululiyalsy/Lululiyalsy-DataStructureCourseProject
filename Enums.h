#pragma once

// 路径规划策略
enum class PathStrategy {
    SHORTEST_DISTANCE,          // 最短距离
    SHORTEST_TIME,              // 最短时间
    MULTI_OBJECTIVE_OPTIMIZATION // 多目标优化
};

// 乘客状态
enum class PassengerState {
    SPAWNED,            // 已生成
    ENTERING,           // 进站中
    TICKETING,          // 购票中
    SECURITY_CHECK,     // 安检中
    MOVING_TO_PLATFORM, // 前往站台
    FROM_TRAIN,         // 列车下车
    ON_PLATFORM,        // 站台等待
    WAITING_TRAIN,      // 候车中
    BOARDING,           // 乘车中
    MOVING_TO_EXIT,     // 前往出口
    EXITING,            // 出站中
    LEFT,               // 已离开
    IN_QUEUE,           // 排队中
    PATH_FOLLOWING,     // 路径跟随
    IN_TRANSIT,         // 通道中
    WAITING_EDGE,       // 等待通道
    REPATHING,          // 重新规划
    COLLIDING           // 碰撞等待
};