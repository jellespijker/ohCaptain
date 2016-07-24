//
// Created by peer23peer on 7/16/16.
//

#include "../../include/Core/Vessel.h"

namespace oCpt {

    iVessel::iVessel() {}

    iVessel::iVessel(iController::ptr controller) {}

    iVessel::~iVessel() {}

    Vessel::Vessel() {
        world_ = World::ptr(new World());
        captain_ = Captain::ptr(new Captain(world_));
        boatswain_ = Boatswain::ptr(new Boatswain(controller_));
    }

    Vessel::Vessel(iController::ptr controller)
            : controller_(controller) {
        world_ = World::ptr(new World());
        captain_ = Captain::ptr(new Captain(world_));
        boatswain_ = Boatswain::ptr(new Boatswain(controller_));
    }

    Vessel::~Vessel() {}

    void Vessel::initialize() {
        boatswain_->initialize();
        captain_->initialize();
    }

    void Vessel::run() {
        std::thread bs_thread(boost::bind(&iBoatswain::run, boatswain_));
        bs_thread.detach();
        std::thread c_thread(boost::bind(&iCaptain::run, captain_));
        c_thread.detach();
        while(1) {}
        //TODO make a work queue
    }

    void Vessel::stop() {

        //TODO stop the threads

    }

}