//
// Created by peer23peer on 7/16/16.
//

#include "../../include/Core/Vessel.h"

namespace oCpt {

    iVessel::iVessel() { }

    iVessel::iVessel(iController::ptr controller) {}

    iVessel::~iVessel() {}

    const boost::shared_ptr<bool> &iVessel::getStopThread() const {
        return stopThread_;
    }

    void iVessel::setStopThread(const boost::shared_ptr<bool> &stopThread) {
        iVessel::stopThread_ = stopThread;
    }

    Vessel::Vessel() {
        world_ = World::ptr(new World());
        captain_ = Captain::ptr(new Captain(world_));
        boatswain_ = Boatswain::ptr(new Boatswain(controller_));

        stopThread_ = boost::shared_ptr<bool>( new bool{false});
        captain_->setStopThread_(stopThread_);
        boatswain_->setStopThread(stopThread_);
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
        std::thread bs_thread(boost::bind(&iBoatswain::run, boatswain_)); //TODO write thread wrapper
        captain_->run();
        bs_thread.join();
        //TODO make a work queue
    }

    void Vessel::stop() {
        *stopThread_ = true;
    }

}