//
// Created by peer23peer on 10/14/16.
//
#pragma once

#include "../Core/Sensor.h"

namespace oCpt {
    namespace components {
        namespace sensors {
            class Razor : public Sensor {
            public:
                typedef struct ReturnValue {
                    double gyro[3];
                    double mag[3];
                    double acc[3];
                } ReturnValue_t;

                Razor(iController::ptr controller, World::ptr world, std::string id, std::string device, unsigned int baudrate);

                ~Razor();

                void updateSensor();

                void run();

                void stop();

                void init();

                void setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice);

            private:
                std::string device_;
                protocol::Serial::ptr serial_;
            };
        }
    }
}
