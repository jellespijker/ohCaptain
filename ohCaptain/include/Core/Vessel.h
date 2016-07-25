//
// Created by peer23peer on 7/16/16.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <vector>
#include <thread>

#include "World.h"
#include "Controller.h"
#include "Captain.h"
#include "Boatswain.h"
#include "Actuator.h"

namespace oCpt {
    class iVessel {
    public:
        typedef boost::shared_ptr<iVessel> ptr;

        iVessel();

        iVessel(iController::ptr controller);

        virtual ~iVessel();

        virtual void initialize() = 0;

        virtual void run() = 0;

        virtual void stop() = 0;
    protected:
        boost::shared_ptr<bool> stopThread_;
    public:
        const boost::shared_ptr<bool> &getStopThread() const;

        void setStopThread(const boost::shared_ptr<bool> &stopThread);
    };

    class Vessel : public iVessel {
    public:
        Vessel();

        Vessel(iController::ptr controller);

        virtual ~Vessel();

        virtual void initialize() override;

        virtual void run() override;

        virtual void stop() override;

    protected:
        World::ptr world_;
        iController::ptr controller_;
        iCaptain::ptr captain_;
        iBoatswain::ptr boatswain_;
        std::vector<iSensor::ptr> sensors_;
        std::vector<iActuator::ptr> actuators_;
    };

}
