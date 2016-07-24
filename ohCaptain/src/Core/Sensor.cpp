//
// Created by peer23peer on 7/15/16.
//

#include "../../include/Core/Sensor.h"
#include "../../include/Core/Boatswain.h"

namespace oCpt {

    iSensor::iSensor(iController::ptr controller,  World::ptr world, std::string id, std::string typeOfSensor)
            : controller_(controller),
              world_(world), id_(id),
              typeOfSensor_(typeOfSensor),
              timer_(0) {
        state_.Value._longlong_t = 0;
    }

    iSensor::~iSensor() {}

    const boost::posix_time::milliseconds &iSensor::getTimer() const {
        return timer_;
    }

    void iSensor::setTimer(const boost::posix_time::milliseconds &timer) {
        iSensor::timer_ = timer;
    }

    const iSensor::signal_t &iSensor::getSig_() const {
        return sig_;
    }

    const iSensor::State &iSensor::getState() const {
        return state_;
    }

    bool iSensor::operator==(iSensor::ptr rhs) {
        return (id_.compare(rhs->id_) == 0 && typeOfSensor_.compare(rhs->typeOfSensor_) == 0 && controller_ == rhs->controller_);
    }

    Sensor::Sensor(iController::ptr controller, World::ptr world, std::string id, std::string typeOfSensor)
            : iSensor(controller, world, id, typeOfSensor) {

    }

    Sensor::~Sensor() {

    }

    void Sensor::updateSensor() {

    }

    void Sensor::run() {

    }

    void Sensor::stop() {

    }
}