//
// Created by peer23peer on 7/15/16.
//

#include "../../include/Core/Controller.h"
#include "../../include/Core/Exception.h"

#include <thread>

#include <boost/make_shared.hpp>

namespace oCpt {

    namespace protocol {

        userspace::userspace() {}

        userspace::~userspace() {}

        bool userspace::modLoaded(std::string modName) {
            //TODO check if Mutex is needed, probably

            // if the module name to be checked is empty return true
            if (modName.compare("") == 0) { return true; }
            std::ifstream fs(MODULE_PATH); //open the module path
            std::string line;
            // Read the lines and check if the module name is in one of them. If this is the case, close the file and return true
            while (std::getline(fs, line)) {
                if (line.find(modName, 0) != std::string::npos) {
                    fs.close();
                    return true;
                }
            }
            // The complete file has been run through without a hit, close the file and return false
            fs.close();
            return false;
        }

        bool userspace::fileExist(std::string fileName) {
            // Check the file stat of Linux
            struct stat buffer;
            return (stat(fileName.c_str(), &buffer) == 0);
        }

        bool userspace::dtboLoaded(std::string dtboName) {
            //Open the cape manager path
            std::ifstream fs(BBB_CAPE_MNGR);
            std::string line;
            // check if the device tree overlay name is present. If that is the case close the file and return true
            while (std::getline(fs, line)) {
                if (line.find(dtboName, 0) != std::string::npos) {
                    fs.close();
                    return true;
                }
            }
            // The complete file has been checked and no hit, close it and return false
            fs.close();
            return false;
        }

        adc::adc(uint8_t id, uint8_t device, std::string modName) {
            device_ = device;
            id_ = id;
            //construct the complete path, from the device and pin ID
            std::stringstream ss;
            ss << ADC_IO_BASE_PATH << std::to_string(device_) << ADC_VOLTAGE_PATH << std::to_string(id_)
               << ADC_VOLTAGE_SUB_PATH;
            path_ = ss.str();
            // Check if necessary module is loaded
            if (!modLoaded(modName)) {
                throw oCptException("Exception! Module not loaded", 0);
            }
            // Check if the path is correct
            if (!fileExist(path_)) {
                throw oCptException("Exception! Userspacefile doesn't exist", 1);
            }
        }

        adc::~adc() {}

        uint16_t &adc::getValue() {
            //TODO check if Mutex is needed for a read operation
            //Open the file, read the value and close if
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
                       Serial::character_size_t csize,
                       Serial::flow_control_t flow,
                       Serial::stop_bits_t stop,
                       unsigned int maxreadlentgh)
                : device_(device),
                  baudrate_(baudrate),
                  ioservice_(ioservice),
                  parity_(parity),
                  csize_(csize),
                  flow_(flow),
                  stop_(stop),
                  serialport_(*ioservice.get(), device_),
                  receivedMsg_(""),
                  maxReadLength_(maxreadlentgh) {
            callback_ = boost::bind(&Serial::internalCallback, this, _1, _2);
        }

        void Serial::open() {
            //If the port is closed, try opening it
            if (!isOpen()) {
                try {
                    serialport_ = serialport_t(*ioservice_.get(), device_);
                } catch (const std::exception &e) {
                    std::cerr << "Unable to open device: " << device_ << std::endl;
                    throw;
                    //TODO better error handling
                }
            }

            //Set the settings for the specified port
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
            //Close the port if it is open and set the callback to the internal closeCallback if needed
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
                serialport_.async_read_some(boost::asio::buffer(read_msg, maxReadLength_),
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
            //TODO check if it is necceasry to resend the msg receive signal if there are still msgs in the que
            return msg;
        }

        void Serial::internalCallback(const unsigned char *data, size_t size) {
            //Create an stringstream and set the pointer for its buffer towards the obtained char buffer
            std::istringstream str;
            str.rdbuf()->pubsetbuf(reinterpret_cast<char *>(const_cast<unsigned char *>(data)), size);
            //Fill the last obtained strings if needed
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
            //convert the msg string to a vector of chars
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
            bool write_inProgress = !msgQueue_.empty(); //writing in progress when the que isn't empty
            msgQueue_.push_back(msg); // push the message to the back of the que
            if (!write_inProgress) {
                writeStart(); // Send the first message
            }
        }

        void Serial::writeStart() {
            //Write the characters asynchronous and bind the writeComplete function
            boost::asio::async_write(serialport_,
                                     boost::asio::buffer(&msgQueue_.front()[0],
                                                         msgQueue_.front().size()),
                                     boost::bind(&Serial::writeComplete,
                                                 this,
                                                 boost::asio::placeholders::error));

        }

        void Serial::writeComplete(const boost::system::error_code &error) {
            // If writing of a message was oke, remove the message from the write que and perform the next write from the que
            if (!error) {
                msgQueue_.pop_front();
                if (!msgQueue_.empty()) {
                    writeStart();
                }
            } else {
                closeCallback(error);
            }
        }

        void Serial::setMaxReadLength(unsigned int maxReadLength) {
            Serial::maxReadLength_ = maxReadLength;
        }

        int gpio::getPinNumber() const {
            return pinNumber_;
        }

        void gpio::setPinNumber(int pinNumber) {
            //TODO change path, check if path is correct, set the Edge, Direction etc from the new pin number
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

        gpio::gpio(int pinNumber, gpio::Direction direction, gpio::Value value, gpio::Edge edge)
                : pinNumber_(pinNumber),
                  direction_(direction),
                  value_(value),
                  edge_(edge),
                  threadRunning_(false) {
            gpiopath_ = GPIO_BASE_PATH;
            gpiopath_.append("gpio" + std::to_string(pinNumber_));
            exportPin(pinNumber_);
            writePinValue<Direction>(gpiopath_, direction_);
            writePinValue<Edge>(gpiopath_, edge_);
            writePinValue<Value>(gpiopath_, value_);
            cb_ = gpio::cb_func(boost::bind(&gpio::internalCbFunc, this));
            //= boost::bind(&gpio::internalCbFunc);
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
                if ((path.find("gpio", sizeOfBasePath) != std::string::npos) &&
                    (path.find("chip", sizeOfBasePath) == std::string::npos)) {
                    const unsigned int loc = path.find("gpio", sizeOfBasePath) + 4;
                    const std::string nr = path.substr(loc);
                    int pinnumber = std::atoi(nr.c_str());
                    Direction direction = readPinValue<Direction>(pinnumber);
                    Value value = readPinValue<Value>(pinnumber);
                    Edge edge = readPinValue<Edge>(pinnumber);
                    gpio::ptr gpio_ptr(new gpio(pinnumber, direction, value, edge));
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
            writePinValue<Value>(gpiopath_, static_cast<Value>(value_ ^ 1));
            //TODO write optimized function currently around 7kHz
        }

        void gpio::setCallbackFunction(gpio::cb_func cb) {
            cb_ = cb;
        }

        void gpio::internalCbFunc() {
            signalChanged();
        }

        void gpio::waitForEdge() {
            if (direction_ == Direction::OUTPUT) {
                return;
            }
            int fd, i, epollfd, count = 0;
            struct epoll_event ev;
            epollfd = epoll_create(1);
            if (epollfd == -1) {
                //TODO error handling
            }
            std::string path = gpiopath_;
            path.append("/value");
            if ((fd = open(path.c_str(), O_RDONLY | O_NONBLOCK)) == -1) {
                //TODO error handling
            }
            ev.events = EPOLLIN | EPOLLET | EPOLLPRI;
            ev.data.fd = fd;

            if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &ev) == -1) {
                //TODO errror handling
            }

            while (count <= 1) {
                i = epoll_wait(epollfd, &ev, 1, -1);
                if (i == -1) {
                    count = 5;
                    //TODO error handling
                } else {
                    count++;
                }
            }
            cb_();
            close(fd);
        }

        void gpio::waitForEdgeAsync() {
            std::thread poll_thread(boost::bind(&gpio::waitForEdge, this));
            poll_thread.detach();
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