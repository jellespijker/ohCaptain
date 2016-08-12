//
// Created by peer23peer on 7/25/16.
//

#pragma once

#include "../Core/Sensor.h"

#include <boost/geometry.hpp>

namespace oCpt {
    namespace components {
        namespace sensors {
            class NavilockNLX02 : public Sensor {
            public:

                NavilockNLX02(iController::ptr controller, World::ptr world, std::string id, std::string device,
                                              unsigned int baudrate);

                ~NavilockNLX02();

                void updateSensor();

                void run();

                void stop();

                void setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice);

            private:
                std::string device_;
                protocol::Serial::ptr serial_;
                double latitude_;
            };
        }
    }
}
