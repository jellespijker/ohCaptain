//
// Created by peer23peer on 7/25/16.
//

#include "../../include/Core/Communication.h"

namespace oCpt {

    iCom::iCom() {
        ioservice_ = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service());
        critical_ = false;
    }

    iCom::~iCom() {

    }

    void iCom::setReadCallback(const iCom::read_function &handler) {
        readFunc_ = handler;
    }

    Serial::Serial(const std::string &device, unsigned int baud)
            : iCom(),
              device_(device),
              baud_(boost::asio::serial_port_base::baud_rate(baud)) {
        //TODO make pointer safe!!!
        serialPort_ = boost::shared_ptr<boost::asio::serial_port>( new boost::asio::serial_port(*ioservice_.get(), device_) );
        deviceOpen_ = false;
        stopMessaging_ = false;
    }

    Serial::~Serial() {
        if (deviceOpen_) {
            close();
        }
    }

    void Serial::start() {
        if (!deviceOpen_) {
            //TODO throw error
        }
        stopMessaging_ = false;
        readStart();
    }

    void Serial::stop() {
        stopMessaging_ = true;
    }

    void Serial::open() {
        if (!deviceOpen_) {
            try {
                serialPort_->open(device_);
               // serialPort_ = boost::shared_ptr<boost::asio::serial_port>(
               //         new boost::asio::serial_port(ioservice_, device_));
            } catch (const std::exception &e) {
                //TODO write error handler
            }
        }

        if (serialPort_->is_open()) {
            serialPort_->set_option(baud_);
            serialPort_->set_option(parity_);
            serialPort_->set_option(csize_);
            serialPort_->set_option(flow_);
            serialPort_->set_option(stop_);
            deviceOpen_ = true;
        } else {
            deviceOpen_ = false;
            //TODO throw error
        }

    }

    void Serial::close() {
        if (deviceOpen_) {
            ioservice_->post(boost::bind(&Serial::closeCallBack, this, boost::system::error_code()));
        }
    }

    bool Serial::isOpen() {
        return deviceOpen_;
    }

    bool Serial::write(const std::string &msg) {
        if (!deviceOpen_) {
            return false;
        }
        std::vector<unsigned char> vStr; //TODO check speed!
        std::for_each(msg.begin(), msg.end(),[&](auto &c){
           vStr.push_back(c);
        });
        ioservice_->post(boost::bind(&Serial::writeCallback, this, vStr));
        return true;
    }

    void Serial::setSerialProperties(boost::asio::serial_port_base::parity parity,
                                     boost::asio::serial_port_base::character_size csize,
                                     boost::asio::serial_port_base::flow_control flow,
                                     boost::asio::serial_port_base::stop_bits stop) {

        parity_ = parity;
        csize_ = csize;
        flow_ = flow;
        stop_ = stop;

        if (deviceOpen_ || serialPort_->is_open()) {
            serialPort_->set_option(baud_);
            serialPort_->set_option(parity_);
            serialPort_->set_option(csize_);
            serialPort_->set_option(flow_);
            serialPort_->set_option(stop_);
        }
    }

    void Serial::closeCallBack(const boost::system::error_code &error) {
        if (error && (error != boost::asio::error::operation_aborted)) {
            //TODO throw error
        }

        serialPort_->close();
        deviceOpen_ = false;
    }

    void Serial::readStart() {
        if (deviceOpen_ && !stopMessaging_) {
            serialPort_->async_read_some(boost::asio::buffer(readMsg, MAX_READ_LENGTH), boost::bind(&Serial::readComplete, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
        }
    }

    void Serial::readComplete(const boost::system::error_code &error, size_t bytesTransferred) {
        if (!error) {
            if (!readFunc_.empty()) {
                readFunc_(const_cast<unsigned char *>(readMsg), bytesTransferred);
                readStart();
            }
        } else {
            closeCallBack(error);
        }
    }

    void Serial::writeStart() {
        boost::asio::async_write(*serialPort_, boost::asio::buffer(&(writeMsg.front()[0]), writeMsg.front().size()), boost::bind(&Serial::writeComplete, this, boost::asio::placeholders::error));
    }

    void Serial::writeComplete(const boost::system::error_code &error) {
        if (!error) {
            writeMsg.pop_front();
            if (!writeMsg.empty()) {
                writeStart();
            }
        } else {
            closeCallBack(error);
        }
    }

    void Serial::writeCallback(const std::vector<unsigned char>& msg) {
        //TODO check logic of this function
        bool writeInProgress = !writeMsg.empty();
        writeMsg.push_back(msg);
        if (!writeInProgress) {
            writeStart();
        }
    }

    void Serial::setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice) {
        ioservice_ = ioservice;
    }

    bool Serial::isCritical() const {
        return critical_;
    }

    void Serial::setCritical(bool critical_) {
        Serial::critical_ = critical_;
    }

}