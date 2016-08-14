//
// Created by peer23peer on 7/15/16.
//

#include "../../include/Core/Controller.h"
#include "../../include/Core/Exception.h"

#include <fstream>
#include <thread>

#include <boost/make_shared.hpp>

namespace oCpt {

    namespace protocol {

        userspace::userspace() {}

        userspace::~userspace() {}

        bool userspace::modLoaded(std::string modName) {
            //TODO check if Mutex is needed, probably
            if (modName.compare("") == 0) { return true; }
            std::ifstream fs("/proc/modules");
            std::string line;
            while (std::getline(fs, line)) {
                if (line.find(modName, 0) != std::string::npos) {
                    fs.close();
                    return true;
                }
            }
            fs.close();
            return false;
        }

        bool userspace::fileExist(std::string fileName) {
            struct stat buffer;
            return (stat(fileName.c_str(), &buffer) == 0);
        }

        bool userspace::dtboLoaded(std::string dtboName) {
            //TODO make method that determine if the dtbo is loaded
            return true;
        }

        adc::adc(uint8_t id, uint8_t device, std::string modName) {
            device_ = device;
            id_ = id;
            std::stringstream ss;
            ss << "/sys/bus/iio/devices/iio:device" << std::to_string(device_) << "/in_voltage" << std::to_string(id_)
               << "_raw";
            path_ = ss.str();
            if (!modLoaded(modName)) {
                throw oCptException("Exception! Module not loaded", 0);
            }
            if (!fileExist(path_)) {
                throw oCptException("Exception! Userspacefile doesn't exist", 1);
            }
        }

        adc::~adc() {}

        uint16_t &adc::getValue() {
            //TODO check if Mutex is needed for a read operation
            std::ifstream fs;
            fs.open(path_.c_str());
            fs >> value_;
            fs.close();
            return value_;
        }

        bool adc::operator==(const adc &rhs) {
            return (rhs.path_.compare(path_) == 0);
        }

        bool adc::compare(const uint8_t &id, const uint8_t &device) {
            return (id_ == id && device_ == device);
        }

        Serial::Serial(const std::string &device, unsigned int baudrate, io_service_t ioservice,
                       Serial::parity_t parity,
                       Serial::character_size_t csize, Serial::flow_control_t flow, Serial::stop_bits_t stop)
                : device_(device),
                  baudrate_(baudrate),
                  ioservice_(ioservice),
                  parity_(parity),
                  csize_(csize),
                  flow_(flow),
                  stop_(stop),
                  serialport_(*ioservice.get(), device_),
                  receivedMsg_("") {
            callback_ = boost::bind(&Serial::internalCallback, this, _1, _2);
        }

        void Serial::open() {
            if (!isOpen()) {
                try {
                    serialport_ = serialport_t(*ioservice_.get(), device_);
                } catch (const std::exception &e) {
                    std::cerr << "Unable to open device: " << device_ << std::endl;
                    throw;
                }
            }

            serialport_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
            serialport_.set_option(parity_);
            serialport_.set_option(csize_);
            serialport_.set_option(flow_);
            serialport_.set_option(stop_);
        }

        bool Serial::isOpen() {
            return serialport_.is_open();
        }

        void Serial::close() {
            if (serialport_.is_open()) {
                ioservice_->post(boost::bind(&Serial::closeCallback,
                                             this,
                                             boost::system::error_code()));
            }
        }

        void Serial::setReadCallback(cb_func cb_function) {
            callback_ = cb_function;
        }

        void Serial::setIOservice(boost::shared_ptr<boost::asio::io_service> io_ptr) {
            ioservice_ = io_ptr;
        }

        void Serial::closeCallback(const boost::system::error_code &error) {
            if (error && (error != boost::asio::error::operation_aborted)) {
                std::cerr << "Error: " << error.message() << std::endl;
            }
            serialport_.close();
        }

        void Serial::start() {
            if (!isOpen()) {
                throw std::runtime_error("Serial port interface not open");
            }
            ReadStart();

            std::thread io_thread(boost::bind(&boost::asio::io_service::run, ioservice_));
            io_thread.detach();
//            ioservice_->run();
            //TODO check if run io_service should be called from here? threaded? or from boatswain
        }

        void Serial::ReadStart() {
            if (isOpen()) {
                serialport_.async_read_some(boost::asio::buffer(read_msg, MAX_READ_LENGTH),
                                            boost::bind(&Serial::readComplete,
                                                        this,
                                                        boost::asio::placeholders::error,
                                                        boost::asio::placeholders::bytes_transferred));
            }
        }

        void Serial::readComplete(const boost::system::error_code &error, size_t bytes_transferred) {
            if (!error) {
                callback_(const_cast<unsigned char *>(read_msg), bytes_transferred);
                ReadStart();
            } else {
                closeCallback(error);
            }
        }

        std::deque<std::string> *Serial::getReturnMsgQueue() {
            return &returnMsgQueue_;
        }

        std::string Serial::readFiFoMsg() {
            std::string msg = returnMsgQueue_.front();
            returnMsgQueue_.pop_front();
            return msg;
        }

        void Serial::internalCallback(const unsigned char *data, size_t size) {
            std::istringstream str;
            str.rdbuf()->pubsetbuf(reinterpret_cast<char *>(const_cast<unsigned char *>(data)), size);
            if (!receivedMsg_.empty()) {
                std::string appMsg = "";
                std::getline(str, appMsg);
                receivedMsg_.append(appMsg);
                if (receivedMsg_.back() == 13) {
                    receivedMsg_.pop_back();
                    returnMsgQueue_.push_back(receivedMsg_);
                    receivedMsg_ = "";
                    msgRecievedSig();
                } else {
                    return;
                }
            }

            while (std::getline(str, receivedMsg_)) {
                if (receivedMsg_.back() == 13) {
                    receivedMsg_.pop_back();
                    returnMsgQueue_.push_back(receivedMsg_);
                    receivedMsg_ = "";
                    msgRecievedSig();
                }
            }

        }

        bool Serial::write(const std::string &msg) {
            std::vector<unsigned char> msg_vec(msg.begin(), msg.end());
            return write(msg_vec);
        }

        bool Serial::write(const std::vector<unsigned char> &data) {
            if (!isOpen()) {
                return false;
            }
            /*TODO check if this is executed in the same thread
             * (http://www.boost.org/doc/libs/1_42_0/doc/html/boost_asio/reference/io_service/post.html)
             * The io_service::post guarantees that the handler will only be called in a thread in which the run(),
             * run_one(), poll() or poll_one() member functions is currently being invoked.*/
            ioservice_->post(boost::bind(&Serial::writeCallback, this, data));
            return true;
        }

        void Serial::writeCallback(const std::vector<unsigned char> &msg) {
            bool write_inProgress = !msgQueue_.empty();
            msgQueue_.push_back(msg);
            if (!write_inProgress) {
                writeStart();
            }
        }

        void Serial::writeStart() {
            boost::asio::async_write(serialport_,
                                     boost::asio::buffer(&msgQueue_.front()[0],
                                                         msgQueue_.front().size()),
                                     boost::bind(&Serial::writeComplete,
                                                 this,
                                                 boost::asio::placeholders::error));

        }

        void Serial::writeComplete(const boost::system::error_code &error) {
            if (!error) {
                msgQueue_.pop_front();
                if (!msgQueue_.empty()) {
                    writeStart();
                }
            } else {
                closeCallback(error);
            }
        }
    }

    iController::iController(World::ptr world)
            : world_(world) {}

    iController::~iController() {}

    ARM::ARM(World::ptr world)
            : iController(world) {}

    ARM::~ARM() {}

    protocol::adc::ptr ARM::getADC(uint8_t id, uint8_t device) {
        std::for_each(adcVector_.begin(), adcVector_.end(), [&](protocol::adc::ptr &A) {
            if (A->compare(id, device)) { return A; }
        });
        return protocol::adc::ptr();
    }

    std::vector<protocol::adc::ptr> *ARM::getAdcVector() {
        return &adcVector_;
    }
}