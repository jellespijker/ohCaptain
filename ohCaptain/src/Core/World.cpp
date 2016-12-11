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
        return oCpt::World::Location::RoutePoint::ptr(); //TODO implement obtain current Location
    }

    void World::Location::push_back(World::Location::RoutePoint::ptr routePoint) {
        //TODO implement way point log
    }

    std::vector<World::Location::RoutePoint::ptr> World::Location::getLocationHistory() {
        return std::vector<World::Location::RoutePoint::ptr>();
    }

    World::Location::cardinal_direction World::Location::stocd(std::string str) {
        char s;

        /*
         * Get the first character to be and make it lower case
         */
        if (str.at(0) <= 'Z' && str.at(0) >= 'A') {
            s = str.at(0) - ('Z' - 'z');
        } else {
            s = str.at(0);
        }

        return static_cast<cardinal_direction >(s);
    }

    std::string World::Location::gpsPoint::toString() {
        std::ostringstream strs;
        strs << latitude.value << static_cast<char>(latitude.direction - 32) << ", " << longitude.value << static_cast<char>(longitude.direction - 32);
        return strs.str();
    }
}