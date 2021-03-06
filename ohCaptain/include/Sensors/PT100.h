//
// Created by peer23peer on 7/18/16.
//

#pragma once

#include "../Core/Sensor.h"

#include <utility>

#include <boost/units/systems/temperature/celsius.hpp>

namespace oCpt {
    namespace components {
        namespace sensors {
            /*!
             * A standard PT100 sensors
             */
            class PT100 : public Sensor {
            public:
                typedef quantity<celsius::temperature, double> ReturnValue_t; //<! Temperature is in Celsius TODO implement conversion to Kelvin

                PT100(iController::ptr controller, World::ptr world, std::string id, uint8_t pinid, uint8_t device);

                ~PT100();

                void updateSensor();

                void run();

                void stop();

                void init();

                void setCalibrationTemperature(std::pair<ReturnValue_t, ReturnValue_t> temparature,
                                               std::pair<uint16_t, uint16_t> analogeValue);

            private:
                uint16_t _analogeValue;
                uint8_t _device = 0;
                uint8_t _pinid = 0;
                ReturnValue_t _dy_dx = 1.0 * celsius::degree;
                ReturnValue_t _constant = 0.0 * celsius::degree;
            };
        }
    }
}
