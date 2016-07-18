//
// Created by peer23peer on 7/15/16.
//

#include "../../include/Core/World.h"

namespace oCpt {

    oCpt::World::Time::Time() {

    }

    oCpt::World::Time::~Time() {

    }

    const boost::chrono::steady_clock &oCpt::World::Time::getTimeClock() const {
        return _timeClock;
    }

    const boost::chrono::steady_clock::time_point oCpt::World::Time::now() {
        return _timeClock.now();
    }

    oCpt::World::World() {}

    oCpt::World::~World() {

    }

}