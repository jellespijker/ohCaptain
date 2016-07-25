//
// Created by peer23peer on 7/21/16.
//
#include "../../include/Core/Boatswain.h"
#include "../../include/Core/Exception.h"
#include "../../include/Core/Sensor.h"

namespace oCpt {
    iBoatswain::iBoatswain(iController::ptr controller)
            : controller_(controller) {
        localStopThread_ = boost::shared_ptr<bool>(new bool{false});
    }

    iBoatswain::~iBoatswain() {

    }

    const boost::shared_ptr<bool> &iBoatswain::getStopThread() const {
        return stopThread_;
    }

    void iBoatswain::setStopThread(const boost::shared_ptr<bool> &stopThread) {
        iBoatswain::stopThread_ = stopThread;
    }

    Boatswain::Boatswain(iController::ptr controller) : iBoatswain(controller) {

    }

    Boatswain::~Boatswain() {

    }

    void Boatswain::run() {
        std::thread boatswainThreads(boost::bind(&boost::asio::io_service::run, &ioservice_));
        ioservice_.run();
        boatswainThreads.join();
    }

    void Boatswain::stop() {
        *localStopThread_ = true;
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
        if (*stopThread_ || *localStopThread_) {
            return;
        }
        int i = -1;
        std::find_if(timerSensors_.begin(), timerSensors_.end(),[&](iSensor::ptr &S){
            i++;
            return S == sensor;
        } );

        //TODO eliminate drift
        timers_[i] = timerPtr(new boost::asio::deadline_timer(ioservice_, sensor->getTimer()));
        timers_[i]->async_wait(boost::bind(&iSensor::run, sensor));
        timers_[i]->async_wait(boost::bind(&Boatswain::resetTimer, this, sensor));
    }


}