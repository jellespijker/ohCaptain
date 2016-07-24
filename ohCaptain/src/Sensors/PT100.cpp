//
// Created by peer23peer on 7/20/16.
//

#include "../../include/Sensors/PT100.h"
#include "../../include/Core/Sensor.h"
#include <iostream>

namespace oCpt {
    namespace components {
        namespace sensors {
            PT100::PT100(iController::ptr controller, std::string id, uint8_t pinid, uint8_t device)
                    : Sensor(controller, oCpt::World::ptr(), id, "PT100"),
                      _pinid(pinid),
                      _device(device){

            }

            PT100::~PT100() {

            }

            void PT100::updateSensor() {
                _analogeValue = controller_->getAdcVector()->at(0)->getValue();
                        //->getADC(_pinid, _device)->getValue();
                state_.Value._double_t = _dy_dx * _analogeValue + _constant;
                //state_.Stamp = world_->getTime().now();
                //TODO fix time stamp
                sig_();
                std::cout << _analogeValue << std::endl;
            }

            void PT100::setCalibrationTemperature(std::pair<double, double> temparature,
                                                  std::pair<uint16_t, uint16_t> analogeValue) {
                _dy_dx = (temparature.second - temparature.first) / (analogeValue.second - analogeValue.first);
                _constant = temparature.first - _dy_dx * analogeValue.first;
            }

            void PT100::run() {
                updateSensor();
            }

            void PT100::stop() {
                Sensor::stop();
            }
        }
    }
}