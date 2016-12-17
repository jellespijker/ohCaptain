//
// Created by peer23peer on 7/21/16.
//
#include "../../include/Core/Boatswain.h"

namespace oCpt {
    iBoatswain::iBoatswain(iController::ptr controller)
            : controller_(controller) {
        localStopThread_ = boost::shared_ptr<bool>(new bool {false});
        ioservice_ = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service());
    }

    iBoatswain::~iBoatswain() {

    }

    const boost::shared_ptr<bool> &iBoatswain::getStopThread() const {
        return stopThread_;
    }

    void iBoatswain::setStopThread(const boost::shared_ptr<bool> &stopThread) {
        iBoatswain::stopThread_ = stopThread;
    }

    boost::shared_ptr<boost::asio::io_service> &iBoatswain::getIOservice() {
        return ioservice_;
    }

    Boatswain::Boatswain(iController::ptr controller) : iBoatswain(controller) {

    }

    Boatswain::~Boatswain() {

    }

    void Boatswain::run() {
        std::for_each(manualSensors_.begin(), manualSensors_.end(), [](iSensor::ptr &P) {
            P->run();
        });
        ioservice_->run();
    }

    void Boatswain::stop() {
        *localStopThread_ = true;
    }

    void Boatswain::initialize() {

    }

    void Boatswain::registerSensor(iSensor::ptr sensor) {
        /*!
         * If the timer is set for the sensor, create a new timer service, register the sensor with the timer sensors, and set the callback functions to execute the Sensor::run function and the internall resetTimer function.
         * If the timer is not set register the Sonsor with the manual sensor.
         */
        if (sensor->getTimer().total_microseconds() > 0) {
            timerPtr timer(new boost::asio::deadline_timer(*ioservice_.get(), sensor->getTimer()));
            timer->async_wait(boost::bind(&iSensor::run, sensor));
            timer->async_wait(boost::bind(&Boatswain::resetTimer, this, sensor));
            timers_.push_back(timer);
            timerSensors_.push_back(sensor);
        } else {
            sensor->setIOservice(ioservice_);
            manualSensors_.push_back(sensor);
            //TODO implement sensors update polling
            //TODO implement sensor update manual request
        }
    }

    void Boatswain::registerActuator(iActuator::ptr actuator) {
        //TODO implement actuator shit
    }

    void Boatswain::resetTimer(iSensor::ptr sensor) {
        /*!
         * Don't excute if the thread is stopped
         */
        if (*stopThread_ || *localStopThread_) {
            return;
        }
        /*!
         * Find the current index of the sensor, this could maybe optimized by using a mappping list
         */
        int i = -1;
        std::find_if(timerSensors_.begin(), timerSensors_.end(), [&](iSensor::ptr &S) {
            i++;
            return S == sensor;
        });

        /*!
         * Set the new timer. drift isn't taken into account at the current time.
         */
        //TODO eliminate drift
        timers_[i] = timerPtr(new boost::asio::deadline_timer(*ioservice_.get(), sensor->getTimer()));
        timers_[i]->async_wait(boost::bind(&iSensor::run, sensor));
        timers_[i]->async_wait(boost::bind(&Boatswain::resetTimer, this, sensor));
    }

    void Boatswain::registerComm(iComm::ptr comm) {
        comm->setIoservice(ioservice_);
    }

}