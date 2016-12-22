//
// Created by peer23peer on 10/14/16.
//

#include "../../include/Sensors/Razor.h"
#include "../../include/Core/Boatswain.h"

namespace oCpt {
    namespace components {
        namespace sensors {
            using namespace protocol;

            Razor::Razor(iController::ptr controller, World::ptr world, std::string id, std::string device,
                         unsigned int baudrate, Mode mode, uint8_t freq)
                    : Sensor(controller, world, id, device),
                      cb(boost::bind(&Razor::msgHandler, this, _1, _2)),
                      mode_(mode),
                      freq_(freq) {
                serial_ = Serial::ptr(
                        new protocol::Serial(device,
                                             baudrate,
                                             Serial::io_service_t(new boost::asio::io_service()),
                                             Serial::parity_t(Serial::parity_t::none),
                                             Serial::character_size_t(8),
                                             Serial::flow_control_t(Serial::flow_control_t::none),
                                             Serial::stop_bits_t(Serial::stop_bits_t::one), 37));

                ReturnValue_t retVal;
                float val[9] = {0};
                fillReturnValue(retVal, val);
                state_.Value = retVal;
                state_.Stamp = world_->now();
            }

            Razor::~Razor() {
                if (serial_->isOpen()) {
                    serial_->close();
                }
            }

            void Razor::updateSensor() {
                if (mode_ == Mode::REQ) { //TODO work with async timer
                    boost::chrono::milliseconds t = boost::chrono::duration_cast<boost::chrono::milliseconds>(
                            world_->now() - state_.Stamp);
                    if (t.count() >
                        20) { //The highest allowable update frequency 50 [Hz} don't introduce unwanted chatter
                        Sensor::updateSensor();
                        serial_->write("#f");
                    }
                }
            }

            void Razor::run() {
                if (!sensorRunning_) {
                    Sensor::run();
                    serial_->start();
                }
            }

            void Razor::stop() {
                if (sensorRunning_) {
                    Sensor::stop();
                    if (serial_->isOpen()) {
                        serial_->write("#0");   // disable continues mode
                        serial_->close();
                    }
                }
            }

            void Razor::setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice) {
                serial_->setIOservice(ioservice);
                Sensor::setIOservice(ioservice);
            }

            void Razor::init() {
                // TODO implement sensor continues mode
                serial_->open();
                serial_->start();
                serial_->setReadCallback(cb);
                setFreq(freq_);
                setMode(mode_);
                serial_->write("#b");
            }

            void Razor::fillReturnValue(Razor::ReturnValue_t &retVal,
                                        float *values) {
                retVal.acc[0] = values[0] * si::meter_per_second_squared;
                retVal.acc[1] = values[1] * si::meter_per_second_squared;
                retVal.acc[2] = values[2] * si::meter_per_second_squared;
                retVal.mag[0] = values[3] * si::tesla;
                retVal.mag[1] = values[4] * si::tesla;
                retVal.mag[2] = values[5] * si::tesla;
                retVal.gyro[0] = values[6] * si::radian_per_second;
                retVal.gyro[1] = values[7] * si::radian_per_second;
                retVal.gyro[2] = values[8] * si::radian_per_second;
            }

            void Razor::msgHandler(const unsigned char *data, size_t size) {
                if (size != 37) return;
                //std::istringstream str;
                //str.rdbuf()->pubsetbuf(reinterpret_cast<char *>(const_cast<unsigned char *>(data)), size);
                //std::string d = str.str();
                std::vector<char *> d;
                for (size_t i = 0; i < size; i++) {
                    d.push_back(reinterpret_cast<char *>(const_cast<unsigned char *>(&data[i])));
                }
                if (checkLRC(d)) {
                    ReturnValue_t retVal;
                    float *val = new float[9];
                    for (int i = 0; i < 9; i++) {
                        val[i] = *(reinterpret_cast<const float *>((data + (i * 4))));
                    }
                    fillReturnValue(retVal, val);
                    delete[] val;
                    state_.Value = retVal;
                    state_.Stamp = world_->now();
                    sig_();
                }
            }

            bool Razor::checkLRC(std::vector<char *> data) {
                char LRC = 0;
                for (size_t i = 0; i < data.size(); i++) {
                    LRC ^= *data[i];
                }
                if (LRC != *data[data.size() - 1]) {
                    return false;
                }
                return true;
            }

            Razor::Mode Razor::getMode() const {
                return mode_;
            }

            void Razor::setMode(Razor::Mode mode) {
                Razor::mode_ = mode;
                if (mode == Mode::CONT) {
                    serial_->write("#1");
                } else {
                    serial_->write("#0");
                }
            }

            uint8_t Razor::getFreq() const {
                return freq_;
            }

            void Razor::setFreq(uint8_t freq) {
                Razor::freq_ = freq;
                serial_->write("#u" + freq);
            }
        }
    }
}

