//
// Created by peer23peer on 10/14/16.
//
#pragma once

#include "../Core/Sensor.h"

#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/magnetic_flux_density.hpp>

namespace oCpt {
    namespace components {
        namespace sensors {
            class Razor : public Sensor {
            public:
                typedef struct ReturnValue {
                    quantity<si::angular_velocity, float> gyro[3];
                    quantity<si::magnetic_flux_density, float> mag[3];
                    quantity<si::acceleration, float> acc[3];
                } ReturnValue_t;

                enum Mode {
                    CONT = 0,
                    REQ = 1
                };

                Razor(iController::ptr controller, World::ptr world, std::string id, std::string device, unsigned int baudrate, Mode mode = Mode::REQ, uint8_t freq = 50);

                ~Razor();

                void updateSensor();

                void run();

                void stop();

                void init();

                void setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice);

            private:
                std::string device_;
                protocol::Serial::ptr serial_;
                protocol::Serial::cb_func cb;
                Mode mode_;
            public:
                Mode getMode() const;

                void setMode(Mode mode);

                uint8_t getFreq() const;

                void setFreq(uint8_t freq);

            private:
                uint8_t freq_ = 50;

                void fillReturnValue(ReturnValue_t &retVal, float *values);

                void msgHandler(const unsigned char *data, size_t size);

                bool checkLRC(std::vector<char *> data);
            };
        }
    }
}
