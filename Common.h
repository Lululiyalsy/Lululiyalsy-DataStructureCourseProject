#pragma once
#include <string>

struct Point {
    int x; // 横坐标
    int y; // 纵坐标
};
using POINT = Point; // 位置点别名

// 安全字符串转整数（自动清理引号，失败返回默认值）
inline int safeStoi(const std::string& str, int defaultVal = 0) {
    if (str.empty() || str == " " || str == "\"\"") return defaultVal;
    try {
        std::string cleaned = str;
        if (!cleaned.empty() && cleaned.front() == '"') cleaned = cleaned.substr(1);
        if (!cleaned.empty() && cleaned.back() == '"') cleaned.pop_back();
        if (cleaned.empty()) return defaultVal;
        return std::stoi(cleaned);
    }
    catch (...) {
        return defaultVal;
    }
}

// 安全字符串转浮点数（自动清理引号，失败返回默认值）
inline double safeStod(const std::string& str, double defaultVal = 0.0) {
    if (str.empty() || str == " " || str == "\"\"") return defaultVal;
    try {
        std::string cleaned = str;
        if (!cleaned.empty() && cleaned.front() == '"') cleaned = cleaned.substr(1);
        if (!cleaned.empty() && cleaned.back() == '"') cleaned.pop_back();
        if (cleaned.empty()) return defaultVal;
        return std::stod(cleaned);
    }
    catch (...) {
        return defaultVal;
    }
}