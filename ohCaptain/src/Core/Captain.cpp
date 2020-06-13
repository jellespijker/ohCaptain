//
// Created by peer23peer on 7/23/16.
//

#include "../../include/Core/Captain.h"

namespace oCpt {

    Captain::Captain(World::ptr world) : iCaptain(world) {
        localStopThread_ = boost::shared_ptr<bool>(new bool{false});
    }

    Captain::~Captain() {

    }

    void Captain::run() {
        while (!*stopThread_ || !*localStopThread_) {
            sleep(1000);
        }
    }

    void Captain::stop() {
        *localStopThread_ = true;
    }

    void Captain::initialize() {

    }

    iCaptain::iCaptain(World::ptr world) {

    }

    iCaptain::~iCaptain() {

    }

    const boost::shared_ptr<bool> &iCaptain::getStopThread_() const {
        return stopThread_;
    }

    void iCaptain::setStopThread_(const boost::shared_ptr<bool> &stopThread_) {
        iCaptain::stopThread_ = stopThread_;
    }
}