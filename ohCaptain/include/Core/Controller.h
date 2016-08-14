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

#include <string>
#include <vector>
#include <mutex>
#include <stdexcept>
#include <deque>

#include "World.h"

#define MAX_READ_LENGTH 4096

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
