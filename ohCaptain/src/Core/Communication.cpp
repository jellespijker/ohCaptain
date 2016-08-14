//
// Created by peer23peer on 7/25/16.
//

#include <sstream>

#include "../../include/Core/Communication.h"
#include "../../include/Core/Exception.h"

namespace oCpt {

    iComm::iComm(const std::string &id, const std::string &device, World::ptr world, iController::io_t ioservice)
            : ioservice_(ioservice),
              timer_(0),
              id_(id),
              device_(device),
              world_(world) {

    }

    iComm::~iComm() {

    }

    const std::string &iComm::getId() const {
        return id_;
    }

    void iComm::setId(const std::string &id) {
        iComm::id_ = id;
    }

    const std::string &iComm::getTypeOfComm() const {
        return typeOfComm_;
    }

    void iComm::setTypeOfComm(const std::string &typeOfComm) {
        iComm::typeOfComm_ = typeOfComm;
    }

    iComm::Message::ptr iComm::readFiFoMsg() {
        if (!msgQueue_.empty()) {
            Message::ptr retVal = msgQueue_.front();
            msgQueue_.pop_front();
            return retVal;
        }
        return nullptr;
    }

    void iComm::setIoservice(const iController::io_t &ioservice) {
        iComm::ioservice_ = ioservice;
    }

    std::deque<iComm::Message::ptr> *iComm::getMsgQueue() {
        return &msgQueue_;
    }

    LoRa::LoRa(const std::string &id, const std::string &device, World::ptr world, iController::io_t ioservice)
            : iComm(id, device, world, ioservice),
              baudrate_(57600),
              serial_(device_, 57600, ioservice_),
              mod_(LORA),
              freq_(868000000),
              pwr_(14),
              sf_(SF12),
              afcbw_(BW125),
              rxbw_(BW250),
              fdev_(5000),
              prlen_(8),
              crc_(true),
              cr_(CR4_8),
              wdt_(0),
              sync_(12),
              bw_(RBW250),
              sendAllowed_(true),
              proceed_(true),
              listen_(false) {

    }

    LoRa::~LoRa() {

    }

    unsigned long LoRa::calculateDownTime(unsigned int payload) {
        double sf = static_cast<double>(sf_);
        double bw = static_cast<double>(bw_);
        double prlen = static_cast<double>(prlen_);
        double T_sym = std::pow(2, sf) / bw;
        double H = crc_ ? 1.0 : 0.0;
        double T_preamble = (prlen + 4.25) * T_sym;
        double cr = static_cast<double>(cr_);
        double ceil = std::ceil((8 * payload - 4 * sf + 28 + 16 - 20 * H) / (4 * sf));
        double payloadSymbNB = 8;
        payloadSymbNB += std::max(ceil * cr + 4, 0.0);
        double T_payloadSymbNb = payloadSymbNB * T_sym;
        double T_packet = T_preamble + T_payloadSymbNb;
        return static_cast<unsigned long>(T_packet);
    }

    std::string LoRa::bandWidthToString(const LoRa::BandWidth &value) {
        switch (value) {
            case BW250:
                return "250";
            case BW200:
                return "200";
            case BW166_7:
                return "166.7";
            case BW125:
                return "125";
            case BW100:
                return "10";
            case BW83_3:
                return "83.3";
            case BW62_5:
                return "62.5";
            case BW50:
                return "50";
            case BW41_7:
                return "41.7";
            case BW31_3:
                return "31.3";
            case BW25:
                return "25";
            case BW20_8:
                return "20.8";
            case BW15_6:
                return "15.6";
            case BW12_5:
                return "12.5";
            case BW10_4:
                return "10.4";
            case BW7_8:
                return "7.8";
            case BW6_3:
                return "6.3";
            case BW5_2:
                return "5.2";
            case BW3_9:
                return "3.9";
            case BW3_1:
                return "3.1";
            default:
                return "2.6";
        }
    }

    std::string LoRa::codingRateToString(const LoRa::CodingRate &value) {
        switch (value) {
            case CR4_5:
                return "4/5";
            case CR4_6:
                return "4/6";
            case CR4_7:
                return "4/7";
            default:
                return "4/8";
        }
    }

    void LoRa::messageRecieved() {
        Message::ptr msg(new Message(serial_.readFiFoMsg(), world_->now()));
        if (msg->Payload.compare("invalid_param") == 0 && !ignoreWarn_) {
            throw new oCptException("Invalid parameter send to LoRa device");
        } else if (msg->Payload.compare("ok") == 0) {
            //TODO handle ok
        } else {
            if (msg->Payload.find("radio_rx ") != std::string::npos) {
                hexToString(msg->Payload.substr(10), msg->Payload);
                msgQueue_.push_back(msg);
                msgRecievedSig();
            }
        }

        proceed_ = true;
        if (listen_ && msg->Payload.compare("busy") != 0) {
            rx();
        }
    }

    void LoRa::run() {

    }

    void LoRa::stop() {

    }

    void LoRa::initialize() {
        //Init and open Serial port
//        serial_.setReadCallback(boost::bind(&LoRa::messageRecieved, this));
        serial_.msgRecievedSig.connect(boost::bind(&LoRa::messageRecieved, this));
        serial_.open();
        serial_.start();

        //clear UART write buffer
        ignoreWarn_ = true;
        write("\r\n");
        ignoreWarn_ = false;

        //Write radio protocol to LoRa chip
        write(buildRadioCmdString<ModulationMode>(MOD, mod_));
        write(buildRadioCmdString<unsigned long>(FREQ, freq_));
        write(buildRadioCmdString<int8_t>(PWR, pwr_));
        write(buildRadioCmdString<SpreadingFactor>(SF, sf_));
        write(buildRadioCmdString<BandWidth>(AFCBW, afcbw_));
        write(buildRadioCmdString<BandWidth>(RXBW, rxbw_));
        write(buildRadioCmdString<uint>(FDEV, fdev_));
        write(buildRadioCmdString<uint>(PRLEN, prlen_));
        write(buildRadioCmdString<bool>(CRC, crc_));
        write(buildRadioCmdString<CodingRate>(CR, cr_));
        write(buildRadioCmdString<unsigned long>(WDT, wdt_));
        write(buildRadioCmdString<unsigned long>(SYNC, sync_));
        write(buildRadioCmdString<RadioBandWidth>(BW, bw_));

        //Put device in listen mode
        macpause();
        listen_ = true;
        rx();
    }

    void LoRa::sendMessage(iComm::Message msg) {
        if (msg.Stamp < world_->now() && sendAllowed_) {
            //Send the message
            serial_.write(buildMacCmdString<int>(PAUSE));
            serial_.write(buildRadioCmdString(rTX, msg.Payload, NONE));

            //TODO set downtime timer to comply with government send times

            // Put device in listen mode
            rx();
        } else {
            //TODO implement timer released send
        }
    }

    iComm::Message::ptr LoRa::recieveMessage() {
        return nullptr;
    }

    void LoRa::recieveAsyncMessage() {

    }

    void LoRa::stringToHex(const std::string str, std::string &hexStr, const bool capital) {
        hexStr.resize(str.size() * 2);
        static const char a = capital ? 0x40 : 0x60;

        for (size_t i = 0; i < str.size(); i++)
        {
            char c = (str[i] >> 4) & 0xF;
            hexStr[i * 2] = c > 9 ? (c - 9) | a : c | '0';
            hexStr[i * 2 + 1] = (str[i] & 0xF) > 9 ? (str[i] - 9) & 0xF | a : str[i] & 0xF | '0';
        }
//        std::stringstream ss;
//        for (auto c : value) {
//            ss << std::hex << (int)c;
//        }
//        return ss.str();
    }

    void LoRa::write(const std::string &value) {
        proceed_ = false;
        serial_.write(value);
        while (!proceed_) {
            usleep(1000);
        }
    }

    void LoRa::rx() {
        //Put device in listen mode
        serial_.write(buildRadioCmdString<int>(rRX, 0, NONE));
    }

    void LoRa::macpause() {
        serial_.write(buildMacCmdString<int>(PAUSE)); //will pause for ~49 days
        //TODO implement a timer if needed?
    }

    void LoRa::hexToString(const std::string hexStr, std::string &str) {
        str.resize((hexStr.size() + 1) / 2);

        for (size_t i = 0, j = 0; i < str.size(); i++, j++)
        {
            str[i] = (hexStr[j] & '@' ? hexStr[j] + 9 : hexStr[j]) << 4, j++;
            str[i] |= (hexStr[j] & '@' ? hexStr[j] + 9 : hexStr[j]) & 0xF;
        }
    }
}