//
// Created by peer23peer on 7/18/16.
//

#pragma once

#include "../Core/Sensor.h"

#include <utility>

namespace oCpt {
    namespace components {
        namespace sensors {
            class PT100 : public Sensor {
            public:
                typedef double ReturnValue_t;

                PT100(iController::ptr controller, World::ptr world, std::string id, uint8_t pinid, uint8_t device);

                ~PT100();

                void updateSensor();

                void run();

                void stop();

                void init();

                void setCalibrationTemperature(std::pair<double, double> temparature,
                                               std::pair<uint16_t, uint16_t> analogeValue);

            private:
                uint16_t _analogeValue;
                uint8_t _device = 0;
                uint8_t _pinid = 0;
                double _dy_dx = 1.0;
                double _constant = 0.0;
            };
        }
    }
}
