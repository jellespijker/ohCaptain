//
// Created by peer23peer on 7/15/16.
//

#include "../../include/Core/Controller.h"
#include "../../include/Core/Exception.h"

#include <fstream>
#include <thread>

#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>

namespace oCpt {

    namespace protocol {

        userspace::userspace() {}

        userspace::~userspace() {}

        bool userspace::modLoaded(std::string modName) {
            //TODO check if Mutex is needed, probably
            if (modName.compare("") == 0) { return true; }
            std::ifstream fs(MODULE_PATH);
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
            std::ifstream fs(BBB_CAPE_MNGR);
            std::string line;
            while (std::getline(fs, line)) {
                if (line.find(dtboName, 0) != std::string::npos) {
                    fs.close();
                    return true;
                }
            }
            fs.close();
            return false;
        }

        adc::adc(uint8_t id, uint8_t device, std::string modName) {
            device_ = device;
            id_ = id;
            std::stringstream ss;
            ss << ADC_IO_BASE_PATH << std::to_string(device_) << ADC_VOLTAGE_PATH << std::to_string(id_)
               << ADC_VOLTAGE_SUB_PATH;
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

        int gpio::getPinNumber() const {
            return pinNumber_;
        }

        void gpio::setPinNumber(int pinNumber) {
            gpio::pinNumber_ = pinNumber;
        }

        gpio::Value gpio::getValue() const {
            if (direction_ == Direction::INPUT) {
                return readPinValue<Value>(pinNumber_);
            }
            return value_;
        }

        void gpio::setValue(gpio::Value value) {
            if (direction_ == Direction::OUTPUT) {
                writePinValue<Value>(gpiopath_, value);
            }
            gpio::value_ = value;
        }

        gpio::Direction gpio::getDirection() const {
            return direction_;
        }

        void gpio::setDirection(gpio::Direction direction) {
            writePinValue<Direction>(gpiopath_, direction_);
            gpio::direction_ = direction;
        }

        gpio::Edge gpio::getEdge() const {
            if (direction_ == Direction::INPUT) {
                return readPinValue<Edge>(gpiopath_);
            }
            return edge_;
        }

        void gpio::setEdge(gpio::Edge edge) {
            if (direction_ == Direction::OUTPUT) {
                writePinValue<Edge>(gpiopath_, edge);
            }
            gpio::edge_ = edge;
        }

        gpio::gpio(int pinNumber, gpio::Direction direction,  gpio::Value value, gpio::Edge edge)
                : pinNumber_(pinNumber),
                  direction_(direction),
                  value_(value),
                  edge_(edge){
            gpiopath_ = GPIO_BASE_PATH;
            gpiopath_.append("gpio" + std::to_string(pinNumber_));
            exportPin(pinNumber_);
            writePinValue<Direction>(gpiopath_, direction_);
            writePinValue<Edge>(gpiopath_, edge_);
            writePinValue<Value>(gpiopath_, value_);
        }

        gpio::~gpio() {
            unexportPin(pinNumber_);
        }

        std::vector<gpio::ptr> gpio::exportedGpios() {
            std::vector<gpio::ptr> gpios;
            boost::filesystem::path p(GPIO_BASE_PATH);
            const unsigned int sizeOfBasePath = p.string().size();

            //! Iterate through all exported pins
            boost::filesystem::directory_iterator end_itr;
            for (boost::filesystem::directory_iterator itr(p); itr != end_itr; ++itr) {
                std::string path = itr->path().string();
                if ((path.find("gpio", sizeOfBasePath) != std::string::npos) && (path.find("chip", sizeOfBasePath) == std::string::npos)) {
                    const unsigned int loc = path.find("gpio",sizeOfBasePath) + 4;
                    const std::string nr = path.substr(loc);
                    int pinnumber = std::atoi(nr.c_str());
                    Direction direction = readPinValue<Direction>(pinnumber);
                    Value value = readPinValue<Value>(pinnumber);
                    Edge edge = readPinValue<Edge>(pinnumber);
                    gpio::ptr gpio_ptr( new gpio(pinnumber, direction, value, edge));
                    gpios.push_back(gpio_ptr);
                }
            }
            return gpios;
        }

        void gpio::exportPin(const int &number) {
            const std::string exportPath = std::string(GPIO_BASE_PATH) + "export";
            std::ofstream fs;
            try {
                fs.open(exportPath);
            } catch (std::ofstream::failure const &ex) {
                //TODO error handling
                return;
            }
            fs << std::to_string(number) << std::endl;
            fs.close();
            usleep(250000);
        }

        void gpio::unexportPin(const int &number) {
            const std::string exportPath = std::string(GPIO_BASE_PATH) + "unexport";
            std::ofstream fs;
            try {
                fs.open(exportPath);
            } catch (std::ofstream::failure const &ex) {
                //TODO error handling
                return;
            }
            fs << std::to_string(number) << std::endl;
            fs.close();
            usleep(250000);
        }

        void gpio::toggle() {
            //TODO write optimized function currently around 7kHz
            if (value_ == Value::HIGH) {
                value_ = Value::LOW;
                writePinValue<Value>(gpiopath_, Value::LOW);
            } else {
                value_ = Value::HIGH;
                writePinValue<>(gpiopath_, Value::HIGH);
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