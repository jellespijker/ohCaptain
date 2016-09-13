//
// Created by peer23peer on 7/15/16.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/system/error_code.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/signals2.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <vector>
#include <mutex>
#include <stdexcept>
#include <deque>

#include "World.h"

#define MAX_READ_LENGTH 4096

#define BBB_CAPE_MNGR "/sys/devices/platform/bone_capemgr/slots"

#define GPIO_BASE_PATH "/sys/class/gpio/"

#define ADC_IO_BASE_PATH "/sys/bus/iio/devices/iio:device"
#define ADC_VOLTAGE_PATH "/in_voltage"
#define ADC_VOLTAGE_SUB_PATH "_raw"

#define MODULE_PATH "/proc/modules"

namespace oCpt {

    namespace protocol {

        class userspace {
        public:
            userspace();

            virtual ~userspace();

        protected:
            bool modLoaded(std::string modName);

            bool fileExist(std::string fileName);

            bool dtboLoaded(std::string dtboName);

            std::mutex usMutex;
        };

        class adc : public userspace {
        public:
            typedef boost::shared_ptr<adc> ptr;

            adc(uint8_t id, uint8_t device, std::string modName = "");

            virtual ~adc();

            uint16_t &getValue();

            bool operator==(const adc &rhs);

            bool compare(const uint8_t &id, const uint8_t &device = 0);

        private:
            uint8_t id_ = 0;
            uint8_t device_ = 0;
            std::string path_ = "";
            uint16_t value_ = 0;
        };

        class gpio : public userspace {
        public:
            typedef boost::shared_ptr<gpio> ptr;
            typedef boost::signals2::signal<void()> signal_t;
            typedef std::function<void()> cb_func;

            enum Direction {INPUT = 105, OUTPUT = 111};
            enum Value { LOW = 48, HIGH = 49};
            enum Edge { NONE = 110, RISING = 114, FALLING = 102, BOTH = 98};

            gpio(int pinNumber, Direction direction = INPUT, Value value = LOW, Edge edge = NONE);

            ~gpio();

            int getPinNumber() const;

            void setPinNumber(int pinNumber);

            Value getValue() const;

            void setValue(Value value);

            Direction getDirection() const;

            void setDirection(Direction direction);

            Edge getEdge() const;

            void setEdge(Edge edge);

            void setCallbackFunction(cb_func cb);

            void waitForEdge();

            void waitForEdgeAsync();

            static std::vector<ptr> exportedGpios();

            void toggle();

            signal_t signalChanged;

        private:
            int pinNumber_;
            Value value_;
            Direction direction_;
            Edge edge_;
            std::string gpiopath_;
            cb_func cb_;
            bool threadRunning_;

            void internalCbFunc();

            void exportPin(const int &number);

            void unexportPin(const int &number);

            template<typename T>
            static T readPinValue(const int &number) {
                std::string path = GPIO_BASE_PATH;
                path.append("gpio" + std::to_string(number));
                return readPinValue<T>(path);
            }

            template<typename T>
            static T readPinValue(std::string path) {
                T retVal;
                if (std::is_same<T, Value>::value) {
                    path.append("/value");
                } else if (std::is_same<T, Direction>::value) {
                    path.append("/direction");
                } else if (std::is_same<T, Edge>::value) {
                    path.append("/edge");
                }
                std::ifstream fs(path);
                char c;
                fs.get(c);
                retVal = static_cast<T>(c);
                fs.close();
                return retVal;
            }

            template<typename T>
            void writePinValue(const int &number, const T &value) {
                std::string path = GPIO_BASE_PATH;
                path.append("gpio" + std::to_string(number));
                writePinValue<T>(path, value);
            }

            template<typename T>
            void writePinValue(std::string path, const T &value) {
                if (std::is_same<T, Value>::value) {
                    path.append("/value");
                    std::ofstream fs(path);
                    fs << (value - 48);
                    fs.close();
                    return;
                } else if (std::is_same<T, Direction>::value) {
                    path.append("/direction");
                    std::ofstream fs(path);
                    switch (value) {
                        case Direction::OUTPUT:
                            fs << "out";
                            break;
                        case Direction::INPUT:
                            fs << "in";
                            break;
                    }
                    fs.close();
                    return;
                } else if (std::is_same<T, Edge>::value) {
                    path.append("/edge");
                    std::ofstream fs(path);
                    switch (value) {
                        case Edge::NONE:
                            fs << "none";
                            break;
                        case Edge::RISING:
                            fs << "edge";
                            break;
                        case Edge::FALLING:
                            fs << "falling";
                            break;
                        case Edge::BOTH:
                            fs << "both";
                            break;
                    }
                    fs.close();
                }
                return;
            }

        };

        class Serial : public userspace {
        public:
            typedef boost::shared_ptr<Serial> ptr;
            typedef std::function<void(const unsigned char *, size_t)> cb_func;
            typedef boost::asio::serial_port_base::parity parity_t;
            typedef boost::asio::serial_port_base::character_size character_size_t;
            typedef boost::asio::serial_port_base::flow_control flow_control_t;
            typedef boost::asio::serial_port_base::stop_bits stop_bits_t;
            typedef boost::shared_ptr<boost::asio::io_service> io_service_t;
            typedef boost::asio::serial_port serialport_t;
            typedef boost::signals2::signal<void()> signal_t;

            Serial(const std::string &device, unsigned int baudrate,
                   io_service_t ioservice = io_service_t(new boost::asio::io_service()),
                   parity_t parity = parity_t(parity_t::none),
                   character_size_t csize = character_size_t(8),
                   flow_control_t flow = flow_control_t(flow_control_t::none),
                   stop_bits_t stop = stop_bits_t(stop_bits_t::one));

            void open();

            void start();

            bool isOpen();

            void close();

            bool write(const std::string &msg);

            bool write(const std::vector<unsigned char> &data);

            void setReadCallback(cb_func cb_function);

            void setIOservice(boost::shared_ptr<boost::asio::io_service> io_ptr);

            std::deque<std::string> *getReturnMsgQueue();

            std::string readFiFoMsg();

            signal_t msgRecievedSig;

        protected:
            void internalCallback(const unsigned char *data, size_t size);

            void closeCallback(const boost::system::error_code &error);

            void readComplete(const boost::system::error_code &error, size_t bytes_transferred);

            void writeCallback(const std::vector<unsigned char> &msg);

            void writeStart();

            void writeComplete(const boost::system::error_code &error);

            void ReadStart();

            std::deque<std::vector<unsigned char>> msgQueue_;
            std::deque<std::string> returnMsgQueue_;
            std::string msg_;
            std::string receivedMsg_;
            unsigned char read_msg[MAX_READ_LENGTH];
            cb_func callback_;
            io_service_t ioservice_;
            std::string device_;
            unsigned int baudrate_;
            parity_t parity_;
            character_size_t csize_;
            flow_control_t flow_;
            stop_bits_t stop_;
            serialport_t serialport_;
            bool firstMsg = true;
        };
    }

    class iController {
    public:
        typedef boost::shared_ptr<iController> ptr;
        typedef boost::shared_ptr<boost::asio::io_service> io_t;

        iController(World::ptr world);

        virtual ~iController();

        virtual std::vector<protocol::adc::ptr> *getAdcVector() = 0;

        virtual protocol::adc::ptr getADC(uint8_t id, uint8_t device) = 0;

    protected:
        std::vector<protocol::adc::ptr> adcVector_;
        World::ptr world_;
    };

    class ARM : public iController {
    public:
        ARM(World::ptr world);

        virtual ~ARM();

        virtual std::vector<protocol::adc::ptr> *getAdcVector();

        virtual protocol::adc::ptr getADC(uint8_t id, uint8_t device);
    };
}
