//
// Created by peer23peer on 7/15/16.
//

#include "../../include/Core/World.h"
#include <algorithm>
#include <string>

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

    World::Location::cardinal_direction World::Location::stocd(std::string str) {
        char s;

        if (str.at(0) <= 'Z' && str.at(0) >= 'A') {
            s = str.at(0) - ('Z' - 'z');
        } else {
            s = str.at(0);
        }

        switch (s) {
            case 110: //n
                return North;
                break;
            case 115: //s
                return South;
                break;
            case 119: //w
                return West;
                break;
            case 101: //e
                return  East;
                break;
        }
        return North;
    }
}