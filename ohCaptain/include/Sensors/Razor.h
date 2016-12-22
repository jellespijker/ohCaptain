//
// Created by peer23peer on 10/14/16.
//
#pragma once

#include "../Core/Sensor.h"

#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/magnetic_flux_density.hpp>
#include <boost/units/systems/si/prefixes.hpp>

namespace oCpt {
    namespace components {
        namespace sensors {
            class Razor : public Sensor {
            public:
                typedef quantity<si::magnetic_flux_density, float> Magnetic_flux_density_t;
                typedef quantity<si::angular_velocity, float> Angular_velocity_t;
                typedef quantity<si::acceleration, float> Acceleration_t;

                typedef struct ReturnValue {
                    Angular_velocity_t gyro[3];
                    Magnetic_flux_density_t mag[3];
                    Acceleration_t acc[3];
                } ReturnValue_t;

                enum Mode {
                    CONT = 0,
                    REQ = 1
                };

                Razor(iController::ptr controller, World::ptr world, std::string id, std::string device,
                      unsigned int baudrate, Mode mode = Mode::CONT, uint8_t freq = 50);

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
