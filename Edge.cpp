#include "Edge.h"
#include <cmath>

Edge::Edge() : toIndex(-1), length(10.0), width(2.0), baseVelocity(1.0), isEscalator(false),
    maxConcurrentOccupancy(4), currentOccupancy(0), congestionLevel(0.0) {
    calculateCapacity();
}

void Edge::calculateCapacity() {
    maxConcurrentOccupancy = static_cast<int>(std::ceil(width / 0.5));
    if (maxConcurrentOccupancy < 1) maxConcurrentOccupancy = 1;
}

void Edge::setWidth(double w) { width = w; calculateCapacity(); }

void Edge::addOccupant() const { currentOccupancy++; updateCongestion(); }
void Edge::removeOccupant() const { if (currentOccupancy > 0) currentOccupancy--; updateCongestion(); }

void Edge::updateCongestion() const {
    if (maxConcurrentOccupancy > 0) {
        congestionLevel = static_cast<double>(currentOccupancy) / maxConcurrentOccupancy;
        if (congestionLevel > 1.0) congestionLevel = 1.0;
    }
}

bool Edge::canEnter() const { return tryEnterEdge(); }

double Edge::getPassThroughTime() const {
    double effectiveVelocity = baseVelocity * (isEscalator ? 2.0 : 1.0);
    double congestionPenalty = 1.0 - (congestionLevel * 0.5);
    effectiveVelocity *= congestionPenalty;
    return (effectiveVelocity > 0.001) ? (length / effectiveVelocity) : 999.0;
}