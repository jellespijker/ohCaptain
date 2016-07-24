//
// Created by peer23peer on 7/21/16.
//

#pragma once

#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include "Controller.h"
#include "Sensor.h"
#include "Actuator.h"

namespace oCpt {

    class iBoatswain : public boost::enable_shared_from_this<iBoatswain> {
    public:
        typedef boost::shared_ptr<iBoatswain> ptr;
        typedef boost::shared_ptr<boost::asio::deadline_timer> timerPtr;

        iBoatswain(iController::ptr controller);

        virtual ~iBoatswain();

        virtual void run() = 0;

        virtual void stop() = 0;

        virtual void initialize() = 0;

        virtual void registerSensor(iSensor::ptr sensor) = 0;

        virtual void registerActuator(iActuator::ptr actuator) = 0;

    protected:
        boost::asio::io_service ioservice_;
        iController::ptr controller_;
        std::vector<timerPtr> timers_;
        std::vector<iSensor::ptr> timerSensors_;

        virtual void resetTimer(iSensor::ptr sensor) = 0;
    };

    class Boatswain : public iBoatswain {
    public:
        Boatswain(iController::ptr controller);

        virtual ~Boatswain() override;

        virtual void run() override;

        virtual void stop() override;

        virtual void initialize() override;

        virtual void registerSensor(iSensor::ptr sensor) override;

        virtual void registerActuator(iActuator::ptr actuator) override;
    protected:
        void resetTimer(iSensor::ptr sensor) override;

    };
}