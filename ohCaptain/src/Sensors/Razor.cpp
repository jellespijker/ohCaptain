//
// Created by peer23peer on 10/14/16.
//

#include "../../include/Sensors/Razor.h"
#include "../../include/Core/Boatswain.h"

namespace oCpt {
    namespace components {
        namespace sensors {

            Razor::Razor(iController::ptr controller, World::ptr world, std::string id, std::string device,
                         unsigned int baudrate) : Sensor(controller, world, id, device) {
                serial_ = protocol::Serial::ptr(new protocol::Serial(device, baudrate));

            }

            Razor::~Razor() {

            }

            void Razor::updateSensor() {
                Sensor::updateSensor();
            }

            void Razor::run() {
                Sensor::run();
            }

            void Razor::stop() {
                Sensor::stop();
            }

            void Razor::setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice) {
                Sensor::setIOservice(ioservice);
            }

            void Razor::init() {
                ReturnValue_t retVal;
                double *p;
                for (int i = 0; i < 3; i++) {
                    switch (i) {
                        case 0:
                            p = &retVal.acc[0];
                            break;
                        case 1:
                            p = &retVal.gyro[0];
                            break;
                        case 2:
                            p = &retVal.mag[0];
                            break;
                    }
                    for (int j = 0; j < 3; j++) {
                        *(p + j) = 0.0;
                    }
                }

                state_.Value = static_cast<ReturnValue_t>(retVal);
            }

        }
    }
}
