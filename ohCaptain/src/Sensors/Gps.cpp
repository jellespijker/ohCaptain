#include "../../include/Sensors/Gps.h"

#include <boost/tokenizer.hpp>

#include <string>

namespace oCpt {
    namespace components {
        namespace sensors {
            using namespace protocol;

            Gps::Gps(iController::ptr controller, World::ptr world, std::string id,
                                                std::string device, unsigned int baudrate)
                    : Sensor(controller, world, id, "GPS"){
                serial_ = Serial::ptr( new Serial(
                        device,
                        baudrate,
                        Serial::io_service_t(new boost::asio::io_service()),
                        Serial::parity_t(Serial::parity_t::none),
                        Serial::character_size_t(8),
                        Serial::flow_control_t(Serial::flow_control_t::none),
                        Serial::stop_bits_t(Serial::stop_bits_t::one), 37));

                serial_->msgRecievedSig.connect(boost::bind(&Gps::interpretMsg, this));
            }

            Gps::~Gps() {
                if (serial_->isOpen()) {
                    serial_->close();
                }
            }

            void Gps::updateSensor() {
                Sensor::updateSensor();
            }

            void Gps::run() {
                if (!sensorRunning_) {
                    Sensor::run();
                    serial_->start();
                }
            }

            void Gps::stop() {
                if (sensorRunning_) {
                    Sensor::stop();
                    if (serial_->isOpen()) {
                        serial_->close();
                    }
                }
            }

            void Gps::setIOservice(boost::shared_ptr <boost::asio::io_service> ioservice) {
                Sensor::setIOservice(ioservice);
            }

            void Gps::interpretMsg() {
                std::string msg = serial_->readFiFoMsg();
                boost::char_delimiters_separator<char> sep(",");
                typedef boost::tokenizer<boost::char_delimiters_separator<char>> tokenizer_t;
                tokenizer_t tok(msg, sep);
                if (tok.begin()->compare("$GPGLL") == std::string::npos) return;
                ReturnValue_t ret = CAST(state_.Value, Gps);
                state_.Stamp = world_->now();
                size_t i = 0;
                auto iBegin = tok.begin();
                std::advance(iBegin, 1);
                auto iEnd = iBegin;
                std::advance(iEnd, 4);
                for (auto it = iBegin; it != iEnd; ++it) {
                    switch (i++) {
                        case 0:
                            ret.latitude.value = std::stod(*it);
                            break;
                        case 1:
                            ret.latitude.direction = World::Location::stocd(*it);
                            break;
                        case 2:
                            ret.longitude.value = std::stod(*it);
                            break;
                        case 3:
                            ret.longitude.direction =  World::Location::stocd(*it);
                            break;
                    }
                }
            }
        }
    }
}