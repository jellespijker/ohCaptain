//
// Created by peer23peer on 7/15/16.
//

#include "../../include/Core/World.h"

namespace oCpt {

    oCpt::World::Time::Time() { }

    oCpt::World::Time::~Time() { }

    boost::chrono::steady_clock &oCpt::World::Time::getTimeClock() {
        return timeClock_;
    }

    boost::chrono::steady_clock::time_point oCpt::World::Time::now() {
        return timeClock_.now();
    }

    oCpt::World::World() {}

    oCpt::World::~World() { }

    World::Time &World::getTime() {
        return time_;
    }

}