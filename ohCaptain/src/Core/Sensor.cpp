//
// Created by peer23peer on 7/15/16.
//

#include "../../include/Core/Sensor.h"

#include <algorithm>

namespace oCpt {

    iSensor::iState::iState(const std::string &symbol, const auto &value) {
        _symbol = symbol;
        _value = value;
    }

    iSensor::iState::~iState() {}

    iSensor::Parameter::Parameter(const std::string &symbol, const auto &value) : iState(symbol, value){ }

    iSensor::Parameter::~Parameter() {}

    auto& iSensor::Parameter::getValue() {
        return _value;
    }

    std::string& iSensor::Parameter::getSymbol() {
        return _value;
    }

    iSensor::iSensor(std::string id, Parameters &parameters) {
        for (int i = 0; i < parameters.size(); i++) {
            if (parameters[i]->getSymbol().compare("threadallowed") == 0){
                _threadedAllowed = parameters[i]->getValue();
                parameters.erase(parameters.begin() + i--);
            }
        }
    }

    iSensor::~iSensor() {}

    std::string &iSensor::getID() {
        return _id;
    }

    void iSensor::setID(const std::string &_id) {
        iSensor::_id = _id;
    }


}