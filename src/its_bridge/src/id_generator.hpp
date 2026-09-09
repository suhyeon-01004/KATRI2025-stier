#pragma once
#include <string>

const std::string SEPERATOR = ":";

inline std::string generateId(long intersectionId, long laneId) {
    return std::to_string(intersectionId) + SEPERATOR + std::to_string(laneId);
}

inline std::pair<long, long> decomposeId(std::string id) {
    size_t pos = id.find(SEPERATOR);
    long intersectionId  = stol(id.substr(0, pos));
    long laneId = stol(id.substr(pos + 1));
    return std::make_pair(intersectionId, laneId);
}