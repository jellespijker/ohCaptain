//
// Created by peer23peer on 7/16/16.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <vector>

#include "World.h"
#include "Sensor.h"
#include "Controller.h"

namespace oCpt {

    class iVessel  {
    public:
        typedef boost::shared_ptr<iVessel> ptr;

        iVessel();
        virtual iVessel(iController::ptr controller) = 0;
        virtual ~iVessel();

        virtual World::ptr getWorld() = 0;
        virtual iSensor::ptr getSensor(std::string id) = 0;
        virtual std::vector<iSensor::ptr> getSensors() = 0;
        virtual iController::ptr getController() = 0;
    };

    class Vessel : public iVessel {
    public:
        Vessel();
        virtual Vessel(iController::ptr controller);
        virtual ~Vessel();

        virtual World::ptr getWorld();
        virtual iSensor::ptr getSensor(std::string id);
        virtual std::vector<iSensor::ptr> getSensors();
        virtual iController::ptr getController();

    private:
        World::ptr _world;
        iController::ptr _controller;
        std::vector<iSensor::ptr> _sensors;
    };

}
