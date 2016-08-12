//
// Created by peer23peer on 7/25/16.
//

#include "../../include/Sensors/NavilockNLX02.h"
#include "../../include/Core/Boatswain.h"

namespace oCpt {
    namespace components {
        namespace sensors {

            NavilockNLX02::NavilockNLX02(iController::ptr controller, World::ptr world, std::string id, std::string device,
                                                     unsigned int baudrate) : Sensor(controller, world, id, device) {
                serial_ = protocol::Serial::ptr(new protocol::Serial(device, baudrate));

            }

            NavilockNLX02::~NavilockNLX02() {

            }

            void NavilockNLX02::updateSensor() {

            }

            void NavilockNLX02::run() {
                updateSensor();
                sig_();
            }

            void NavilockNLX02::stop() {
                Sensor::stop();
            }

            void NavilockNLX02::setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice) {
                serial_->setIOservice(ioservice);
            }
        }
    }
}