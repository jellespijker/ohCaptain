//
// Created by peer23peer on 7/15/16.
//

#include "../../include/Core/World.h"

namespace oCpt {

    World::Time::Time() {}

    World::Time::~Time() {}

    World::Time::clock_t &oCpt::World::Time::getTimeClock() {
        return timeClock_;
    }

    World::Time::timepoint_t oCpt::World::Time::now() {
        return timeClock_.now();
    }

    World::World() {

    }

    World::~World() {}

    World::Time &World::getTime() {
        return time_;
    }

    World::Time::timepoint_t World::now() {
        return time_.now();
    }

    World::Location::Location() {

    }

    World::Location::~Location() {

    }

    World::Location::RoutePoint::ptr World::Location::getCurrentLocation(bool newMeasurement) {
        return oCpt::World::Location::RoutePoint::ptr();
    }

    void World::Location::push_back(World::Location::RoutePoint::ptr routePoint) {

    }

    std::vector<World::Location::RoutePoint::ptr> World::Location::getLocationHistory() {
        return std::vector<World::Location::RoutePoint::ptr>();
    }
}