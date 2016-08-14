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
#include "Communication.h"

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

        virtual void registerComm(iComm::ptr comm) = 0;

        const boost::shared_ptr<bool> &getStopThread() const;

        void setStopThread(const boost::shared_ptr<bool> &stopThread);

        boost::shared_ptr<boost::asio::io_service> &getIOservice();

    protected:
        boost::shared_ptr<boost::asio::io_service> ioservice_;
        iController::ptr controller_;
        std::vector<timerPtr> timers_;
        std::vector<iSensor::ptr> timerSensors_;
        std::vector<iSensor::ptr> manualSensors_;
        boost::shared_ptr<bool> stopThread_;
        boost::shared_ptr<bool> localStopThread_;

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

        virtual void registerComm(iComm::ptr comm) override;

    protected:
        void resetTimer(iSensor::ptr sensor) override;

    };
}
