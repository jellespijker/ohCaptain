//
// Created by peer23peer on 7/25/16.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/function.hpp>

#include <string>
#include <vector>
#include <deque>

#define MAX_READ_LENGTH 512

namespace oCpt {
    class iCom {
    public:
        typedef boost::shared_ptr<iCom> ptr;
        typedef boost::function<void (const unsigned char*, size_t)> read_function;

        iCom(boost::shared_ptr<boost::asio::io_service> ioservice);

        virtual ~iCom();

        void setReadCallback(const read_function &handler);

        virtual void start() = 0;

        virtual void stop() = 0;

        virtual void open() = 0;

        virtual void close() = 0;

        virtual bool isOpen() = 0;

        virtual bool write(const std::string &msg) = 0;

    protected:
        read_function readFunc_;
        boost::shared_ptr<boost::asio::io_service> ioservice_;
    };

    class Serial : public iCom {
    public:
        Serial(boost::shared_ptr<boost::asio::io_service> ioservice, const std::string &device, unsigned int baud);

        virtual ~Serial() override;

        virtual void start() override;

        virtual void stop() override;

        virtual void open() override;

        virtual void close() override;

        virtual bool isOpen() override;

        virtual bool write(const std::string &msg) override;

        void setSerialProperties(boost::asio::serial_port_base::parity parity = boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none),
                                 boost::asio::serial_port_base::character_size csize = boost::asio::serial_port_base::character_size(
                                         8),
                                 boost::asio::serial_port_base::flow_control flow = boost::asio::serial_port_base::flow_control(
                                         boost::asio::serial_port_base::flow_control::none),
                                 boost::asio::serial_port_base::stop_bits stop = boost::asio::serial_port_base::stop_bits(
                                         boost::asio::serial_port_base::stop_bits::one));

    protected:
        void closeCallBack(const boost::system::error_code &error);

        void readStart();

        void readComplete(const boost::system::error_code &error, size_t bytesTransferred);

        void writeStart();

        void writeComplete(const boost::system::error_code &error);

        void writeCallback(const std::vector<unsigned char>& msg);

        std::string device_;

        bool deviceOpen_;
        bool stopMessaging_;
        std::deque<std::vector<unsigned char>> writeMsg;
        boost::shared_ptr<boost::asio::serial_port> serialPort_;
        boost::asio::serial_port_base::baud_rate baud_;
        boost::asio::serial_port_base::parity parity_ = boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none);
        boost::asio::serial_port_base::character_size csize_ = boost::asio::serial_port_base::character_size(8);
        boost::asio::serial_port_base::flow_control flow_ = boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none);
        boost::asio::serial_port_base::stop_bits stop_ = boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one);
        unsigned char readMsg[MAX_READ_LENGTH];

    };
}
