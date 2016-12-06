//
// Created by peer23peer on 7/25/16.
//

#pragma once

#include "../Core/Sensor.h"
#include "../Core/Controller.h"

namespace oCpt {
    namespace components {
        namespace sensors {
            class Gps : public Sensor {
            public:
                typedef oCpt::World::Location::gpsPoint_t ReturnValue_t;

                Gps(iController::ptr controller, World::ptr world, std::string id, std::string device,
                              unsigned int baudrate);

                ~Gps();

                void updateSensor();

                void run();

                void stop();

                void setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice);

            protected:
                std::string device_;
                protocol::Serial::ptr serial_;

                void interpretMsg();
            };
        }
    }
}