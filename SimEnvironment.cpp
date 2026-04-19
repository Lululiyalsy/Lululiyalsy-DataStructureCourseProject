#include "SimEnvironment.h"
#include "SubwayGraph.h"
#include "Passenger.h"
#include "Node.h"
#include <iostream>
#include <iomanip>
#include <sstream>

int VirtualClock::get_current_weekday() const {
    int total_days = static_cast<int>(total_sim_seconds / 86400);
    return (start_day_of_week + total_days) % 7;
}

int VirtualClock::get_seconds_today() const {
    return static_cast<int>(total_sim_seconds) % 86400;
}

int VirtualClock::get_current_hour() const {
    return get_seconds_today() / 3600;
}

int VirtualClock::get_current_minute() const {
    return (get_seconds_today() % 3600) / 60;
}

std::string VirtualClock::get_formatted_time() const {
    int weekday = get_current_weekday();
    const char* weekdays[] = { "\u5468\u4e00", "\u5468\u4e8c", "\u5468\u4e09", "\u5468\u56db", "\u5468\u4e94", "\u5468\u516d", "\u5468\u65e5" };
    std::ostringstream oss;
    oss << weekdays[weekday] << " "
        << std::setfill('0') << std::setw(2) << get_current_hour() << ":"
        << std::setfill('0') << std::setw(2) << get_current_minute();
    return oss.str();
}

bool VirtualClock::is_weekday() const {
    int weekday = get_current_weekday();
    return weekday >= 0 && weekday <= 4;
}

bool VirtualClock::is_weekend() const { return !is_weekday(); }

PassengerGenerator::PassengerGenerator(VirtualClock& c, SubwayGraph& graph, int maxOnline)
    : clock(c), graphRef(graph), total_generated(0), trainPassengerCounter(0), maxOnlinePassengers(maxOnline) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    rng.seed(seed);
    initialize_default_schedule();
    updateStationLayout();
}

void PassengerGenerator::updateStationLayout() {
    platformIds.clear();
    for (const auto& node : graphRef.getAllNodes()) {
        if (node->getTypeCode() == "PLATFORM") platformIds.push_back(node->getId());
    }
}

std::vector<Passenger> PassengerGenerator::generateTrainPassengers(double dt, int remaining) {
    std::vector<Passenger> train_passengers;
    if (remaining <= 0) return train_passengers;
    for (const auto& platformId : platformIds) {
        if (remaining <= 0) break;
        PlatformNode* platform = dynamic_cast<PlatformNode*>(graphRef.getNode(platformId));
        if (platform && platform->isTrainArrivingNow() && platform->canAcceptTrainPassengers()) {
            int numPassengers = std::min(8, remaining);
            for (int i = 0; i < numPassengers; ++i) {
                trainPassengerCounter++;
                total_generated++;
                std::string exitId = findRandomExit();
                if (!exitId.empty()) {
                    double currentAbsTime = clock.get_total_seconds();
                    Passenger p(trainPassengerCounter, graphRef.getIndex(platformId), graphRef.getIndex(exitId),
                        generateDefaultAttributes(), currentAbsTime, PathStrategy::SHORTEST_TIME, true, &graphRef);
                    std::vector<int> path = graphRef.findPath(platformId, exitId, PathStrategy::SHORTEST_TIME);
                    if (!path.empty()) {
                        p.setPath(path);
                        train_passengers.push_back(p);
                        remaining--;
                    } else {
                        trainPassengerCounter--;
                        total_generated--;
                    }
                } else {
                    trainPassengerCounter--;
                    total_generated--;
                }
            }
        }
    }
    return train_passengers;
}

std::string PassengerGenerator::findRandomExit() const {
    std::vector<std::string> exitIds;
    for (const auto& node : graphRef.getAllNodes()) {
        if (node->getTypeCode() == "EXIT") exitIds.push_back(node->getId());
    }
    if (!exitIds.empty()) {
        std::uniform_int_distribution<> dist(0, exitIds.size() - 1);
        return exitIds[dist(rng)];
    }
    return "";
}

PassengerAttributes PassengerGenerator::generateDefaultAttributes() const {
    PassengerAttributes attrs;
    std::uniform_real_distribution<float> speed_dist(0.8f, 1.2f);
    std::uniform_real_distribution<float> patience_dist(0.5f, 1.0f);
    std::uniform_real_distribution<float> fam_dist(0.3f, 0.8f);
    std::bernoulli_distribution luggage_dist(0.1f);
    std::bernoulli_distribution purpose_dist(0.8f);
    attrs.speed = speed_dist(rng); attrs.patience = patience_dist(rng);
    attrs.familiarity = fam_dist(rng); attrs.has_luggage = luggage_dist(rng);
    attrs.purpose = purpose_dist(rng) ? "commute" : "leisure";
    return attrs;
}

void PassengerGenerator::initialize_default_schedule() {
    TimeSlot morning_peak;
    morning_peak.start_second = TimeSlot::time_to_seconds(7, 0);
    morning_peak.end_second = TimeSlot::time_to_seconds(9, 0);
    morning_peak.day_mask = TimeSlot::weekday_mask();
    morning_peak.profile.name = "\u5de5\u4f5c\u65e5\u65e9\u9ad8\u5cf0";
    morning_peak.profile.arrival_rate = 8.0;
    morning_peak.profile.familiarity_min = 0.8f; morning_peak.profile.familiarity_max = 1.0f;
    morning_peak.profile.patience_min = 0.4f; morning_peak.profile.patience_max = 0.7f;
    morning_peak.profile.luggage_prob = 0.15f; morning_peak.profile.commute_ratio = 0.95f;
    schedule.push_back(morning_peak);

    TimeSlot evening_peak;
    evening_peak.start_second = TimeSlot::time_to_seconds(17, 0);
    evening_peak.end_second = TimeSlot::time_to_seconds(19, 0);
    evening_peak.day_mask = TimeSlot::weekday_mask();
    evening_peak.profile.name = "\u5de5\u4f5c\u65e5\u665a\u9ad8\u5cf0";
    evening_peak.profile.arrival_rate = 7.0;
    evening_peak.profile.familiarity_min = 0.8f; evening_peak.profile.familiarity_max = 1.0f;
    evening_peak.profile.patience_min = 0.3f; evening_peak.profile.patience_max = 0.6f;
    evening_peak.profile.luggage_prob = 0.2f; evening_peak.profile.commute_ratio = 0.9f;
    schedule.push_back(evening_peak);

    TimeSlot weekday_offpeak;
    weekday_offpeak.start_second = TimeSlot::time_to_seconds(9, 0);
    weekday_offpeak.end_second = TimeSlot::time_to_seconds(17, 0);
    weekday_offpeak.day_mask = TimeSlot::weekday_mask();
    weekday_offpeak.profile.name = "\u5de5\u4f5c\u65e5\u5e73\u5cf0";
    weekday_offpeak.profile.arrival_rate = 3.0;
    weekday_offpeak.profile.familiarity_min = 0.6f; weekday_offpeak.profile.familiarity_max = 0.9f;
    weekday_offpeak.profile.patience_min = 0.6f; weekday_offpeak.profile.patience_max = 0.9f;
    weekday_offpeak.profile.luggage_prob = 0.3f; weekday_offpeak.profile.commute_ratio = 0.5f;
    schedule.push_back(weekday_offpeak);

    TimeSlot weekend_peak;
    weekend_peak.start_second = TimeSlot::time_to_seconds(10, 0);
    weekend_peak.end_second = TimeSlot::time_to_seconds(20, 0);
    weekend_peak.day_mask = TimeSlot::weekend_mask();
    weekend_peak.profile.name = "\u5468\u672b\u9ad8\u5cf0";
    weekend_peak.profile.arrival_rate = 4.0;
    weekend_peak.profile.familiarity_min = 0.3f; weekend_peak.profile.familiarity_max = 0.7f;
    weekend_peak.profile.patience_min = 0.7f; weekend_peak.profile.patience_max = 1.0f;
    weekend_peak.profile.luggage_prob = 0.5f; weekend_peak.profile.commute_ratio = 0.2f;
    schedule.push_back(weekend_peak);

    TimeSlot default_slot;
    default_slot.start_second = 0; default_slot.end_second = 86400;
    default_slot.day_mask = TimeSlot::daily_mask();
    default_slot.profile.name = "\u9ed8\u8ba4\u5e73\u5cf0";
    default_slot.profile.arrival_rate = 2.0;
    schedule.push_back(default_slot);
}

const CrowdProfile* PassengerGenerator::get_current_profile() {
    int weekday = clock.get_current_weekday();
    int sec_today = clock.get_seconds_today();
    for (const auto& slot : schedule) {
        bool day_match = (slot.day_mask & (1 << weekday));
        bool time_match = (sec_today >= slot.start_second && sec_today < slot.end_second);
        if (day_match && time_match) return &slot.profile;
    }
    return &schedule.back().profile;
}

std::vector<Passenger> PassengerGenerator::generateEntryPassengers(double dt, int remaining) {
    std::vector<Passenger> new_passengers;
    const CrowdProfile* profile = get_current_profile();
    if (!profile || remaining <= 0) return new_passengers;
    double rate = profile->arrival_rate;
    std::poisson_distribution<int> dist(rate * dt);
    int count = dist(rng);
    count = std::min(count, remaining);
    if (count > 0) {
        new_passengers.reserve(count);
        std::uniform_real_distribution<float> speed_dist(profile->speed_min, profile->speed_max);
        std::uniform_real_distribution<float> patience_dist(profile->patience_min, profile->patience_max);
        std::uniform_real_distribution<float> fam_dist(profile->familiarity_min, profile->familiarity_max);
        std::bernoulli_distribution luggage_dist(profile->luggage_prob);
        std::bernoulli_distribution purpose_dist(profile->commute_ratio);
        std::vector<std::string> hallIds, exitIds, platformIds;
        for (const auto& node : graphRef.getAllNodes()) {
            if (node->getTypeCode() == "HALL") hallIds.push_back(node->getId());
            else if (node->getTypeCode() == "EXIT") exitIds.push_back(node->getId());
            else if (node->getTypeCode() == "PLATFORM") platformIds.push_back(node->getId());
        }
        if (hallIds.empty() || (exitIds.empty() && platformIds.empty())) return new_passengers;
        std::uniform_int_distribution<> hallDist(0, hallIds.size() - 1);
        std::uniform_int_distribution<> exitDist(0, exitIds.size() - 1);
        std::uniform_int_distribution<> platformDist(0, platformIds.size() - 1);
        std::bernoulli_distribution direction_dist(0.6);
        for (int i = 0; i < count; ++i) {
            total_generated++;
            PassengerAttributes attrs;
            attrs.speed = speed_dist(rng); attrs.patience = patience_dist(rng);
            attrs.familiarity = fam_dist(rng); attrs.has_luggage = luggage_dist(rng);
            attrs.purpose = purpose_dist(rng) ? "commute" : "leisure";
            PathStrategy strategy = PathStrategy::MULTI_OBJECTIVE_OPTIMIZATION;
            if (attrs.has_luggage) strategy = PathStrategy::SHORTEST_DISTANCE;
            else if (purpose_dist(rng) && (clock.get_current_hour() >= 7 && clock.get_current_hour() <= 9)) strategy = PathStrategy::SHORTEST_TIME;
            std::string startId = hallIds[hallDist(rng)];
            std::string endId;
            bool headingToPlatform = direction_dist(rng) && !platformIds.empty();
            if (headingToPlatform) endId = platformIds[platformDist(rng)];
            else if (!exitIds.empty()) endId = exitIds[exitDist(rng)];
            else { endId = platformIds[platformDist(rng)]; headingToPlatform = true; }
            new_passengers.emplace_back(total_generated, graphRef.getIndex(startId), graphRef.getIndex(endId),
                attrs, clock.get_total_seconds(), strategy, false, &graphRef);
            if (headingToPlatform) { new_passengers.back().isFromTrain = false; new_passengers.back().headingToPlatform = true; }
            std::vector<int> path = graphRef.findPath(startId, endId, strategy);
            if (!path.empty()) {
                new_passengers.back().setPath(path);
            } else {
                new_passengers.pop_back();
                total_generated--;
            }
        }
        stats.profile_counts[profile->name] += count;
        if (clock.is_weekday()) { stats.weekday_total += count; if (rate >= 2.0) stats.peak_total += count; else stats.offpeak_total += count; }
        else stats.weekend_total += count;
    }
    return new_passengers;
}

std::vector<Passenger> PassengerGenerator::generate(double dt, int currentOnlineCount) {
    std::vector<Passenger> all_passengers;
    if (currentOnlineCount >= maxOnlinePassengers) return all_passengers;
    int remaining = maxOnlinePassengers - currentOnlineCount;
    std::vector<Passenger> entry_passengers = generateEntryPassengers(dt, remaining);
    all_passengers.insert(all_passengers.end(), entry_passengers.begin(), entry_passengers.end());
    remaining = maxOnlinePassengers - currentOnlineCount - static_cast<int>(all_passengers.size());
    if (remaining <= 0) return all_passengers;
    std::vector<Passenger> train_passengers = generateTrainPassengers(dt, remaining);
    all_passengers.insert(all_passengers.end(), train_passengers.begin(), train_passengers.end());
    return all_passengers;
}

void PassengerGenerator::print_stats() const {
    std::cout << "\n========== \u5ba2\u6d41\u751f\u6210\u7edf\u8ba1 ==========" << std::endl;
    std::cout << "\u603b\u751f\u6210\u4eba\u6570\uff1a" << total_generated << " (\u5165\u53e3\u8fdb\u7ad9: " << (total_generated - trainPassengerCounter) << ", \u5217\u8f66\u4e0b\u8f66: " << trainPassengerCounter << ")" << std::endl;
    std::cout << "\u5de5\u4f5c\u65e5\u5ba2\u6d41\uff1a" << stats.weekday_total << std::endl;
    std::cout << "\u5468\u672b\u5ba2\u6d41\uff1a" << stats.weekend_total << std::endl;
    std::cout << "\u9ad8\u5cf0\u65f6\u6bb5\u5ba2\u6d41\uff1a" << stats.peak_total << std::endl;
    std::cout << "\u5e73\u5cf0\u65f6\u6bb5\u5ba2\u6d41\uff1a" << stats.offpeak_total << std::endl;
    std::cout << "\n\u5404\u65f6\u6bb5\u5206\u5e03\uff1a" << std::endl;
    for (const auto& pair : stats.profile_counts) std::cout << "  " << pair.first << ": " << pair.second << " \u4eba" << std::endl;
    std::cout << "================================" << std::endl;
}