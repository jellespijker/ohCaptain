//
// Created by peer23peer on 7/21/16.
//
#include "../../include/Core/Boatswain.h"
#include "../../include/Core/Exception.h"
#include "../../include/Core/Sensor.h"

namespace oCpt {
    iBoatswain::iBoatswain(iController::ptr controller)
            : controller_(controller) {

    }

    iBoatswain::~iBoatswain() {

    }

    Boatswain::Boatswain(iController::ptr controller) : iBoatswain(controller) {

    }

    Boatswain::~Boatswain() {

    }

    void Boatswain::run() {
        //std::thread boatswainThreads(boost::bind(&Boatswain::runIO, this));
        //boatswainThreads.detach();
        ioservice_.run();
    }

    void Boatswain::stop() {
        //TODO write stop function
    }

    void Boatswain::initialize() {

    }

    void Boatswain::registerSensor(iSensor::ptr sensor) {
        if (sensor->getTimer().total_microseconds() > 0) {
            timerPtr timer(new boost::asio::deadline_timer(ioservice_, sensor->getTimer()));
            timer->async_wait(boost::bind(&iSensor::run, sensor));
            timer->async_wait(boost::bind(&Boatswain::resetTimer, this, sensor));
            timers_.push_back(timer);
            timerSensors_.push_back(sensor);
        } else {
            //TODO implement sensors update polling
            //TODO implement sensor update manual request
        }
    }

    void Boatswain::registerActuator(iActuator::ptr actuator) {

    }

    void Boatswain::resetTimer(iSensor::ptr sensor) {
        int i = 0;
        std::for_each(timerSensors_.begin(), timerSensors_.end(),[&](iSensor::ptr &P){
            if (P == sensor) {
                return;
            }
            i++;
        });
        //TODO eliminate drift
        timers_[i] = timerPtr(new boost::asio::deadline_timer(ioservice_, sensor->getTimer()));
        timers_[i]->async_wait(boost::bind(&iSensor::run, sensor));
        timers_[i]->async_wait(boost::bind(&Boatswain::resetTimer, this, sensor));
    }


}