//
// Created by peer23peer on 7/25/16.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio.hpp>

#include <string>
#include <vector>
#include <deque>
#include <typeinfo>

#include "Controller.h"
#include "World.h"
#include "Exception.h"

namespace oCpt {

    class iComm {
    public:
        typedef boost::shared_ptr<iComm> ptr;
        typedef boost::signals2::signal<void()> signal_t;

        struct Message {
            typedef boost::shared_ptr<Message> ptr;

            Message(const std::string &payload, const World::Time::timepoint_t &stamp)
                    : Payload(payload),
                      Stamp(stamp) {};

            ~Message() {};
            std::string Payload;
            World::Time::timepoint_t Stamp;
        };

        iComm(const std::string &id, const std::string &device, World::ptr world = World::ptr(new World()),
              iController::io_t ioservice = iController::io_t(new boost::asio::io_service()));

        virtual ~iComm();

        virtual void run() = 0;

        virtual void stop() = 0;

        virtual void initialize() = 0;

        virtual void sendMessage(Message msg) = 0;

        virtual Message::ptr recieveMessage() = 0;

        virtual void recieveAsyncMessage() = 0;

        const std::string &getId() const;

        void setId(const std::string &id);

        const std::string &getTypeOfComm() const;

        void setTypeOfComm(const std::string &typeOfComm);

        Message::ptr readFiFoMsg();

        std::deque<Message::ptr> *getMsgQueue();

        void setIoservice(const iController::io_t &ioservice);

        signal_t msgRecievedSig;
    protected:
        std::string id_;
        std::string typeOfComm_;
        std::string device_;
        boost::posix_time::milliseconds timer_;
        std::deque<Message::ptr> msgQueue_;
        iController::io_t ioservice_;
        World::ptr world_;
    };

    class LoRa : public iComm {
    public:

        enum ModulationMode {
            LORA = 1,
            FSK = 2
        };

        enum SpreadingFactor {
            SF7 = 7,
            SF8 = 8,
            SF9 = 9,
            SF10 = 10,
            SF11 = 11,
            SF12 = 12
        };

        enum BandWidth {
            BW250 = 1,
            BW200 = 2,
            BW166_7 = 3,
            BW125 = 4,
            BW100 = 5,
            BW83_3 = 6,
            BW62_5 = 7,
            BW50 = 8,
            BW41_7 = 9,
            BW31_3 = 10,
            BW25 = 11,
            BW20_8 = 12,
            BW15_6 = 13,
            BW12_5 = 14,
            BW10_4 = 15,
            BW7_8 = 16,
            BW6_3 = 17,
            BW5_2 = 18,
            BW3_9 = 19,
            BW3_1 = 20,
            BW2_6 = 21
        };

        enum CodingRate {
            CR4_5 = 4,
            CR4_6 = 3,
            CR4_7 = 2,
            CR4_8 = 1
        };

        enum RadioBandWidth {
            RBW500 = 500,
            RBW250 = 250,
            RBW125 = 125
        };

        enum GetSet {
            GET,
            SET,
            NONE
        };

        enum RadioCommand {
            MOD,
            FREQ,
            PWR,
            SF,
            AFCBW,
            RXBW,
            FSKBITRATE,
            FDEV,
            PRLEN,
            CRC,
            CR,
            WDT,
            SYNC,
            BW,
            rRX,
            rTX
        };

        enum MacCommand {
            PAUSE,
            RESET,
            mTX,
            JOIN,
            SAVE,
            FORCEENABLE,
            RESUME
        };

        LoRa(const std::string &id, const std::string &device, World::ptr world = World::ptr(new World()),
             iController::io_t ioservice = iController::io_t(new boost::asio::io_service()));

        virtual ~LoRa();

        virtual void run() override;

        virtual void stop() override;

        virtual void initialize() override;

        virtual void sendMessage(Message msg) override;

        virtual Message::ptr recieveMessage() override;

        virtual void recieveAsyncMessage() override;

    protected:
        void messageRecieved();

        std::string bandWidthToString(const BandWidth &value);

        std::string codingRateToString(const CodingRate &value);

        void stringToHex(const std::string str, std::string &hexStr, const bool capital = true);

        void hexToString(const std::string hexStr, std::string &str);

        template<typename T>
        std::string encodeTypeToHex(T value, bool capital = true){
            std::string retVal;
            retVal.resize(sizeof(T) * 2);
            static const char a = capital ? 0x40 : 0x60;
            char *begin = reinterpret_cast<char *>(&value);
            for (uint8_t i = 0; i < sizeof(T); i++) {
                unsigned char p = *(begin + i);
                char c = (p >> 4) & 0xF;
                retVal[i * 2] = c > 9 ? (c - 9) | a : c | '0';
                retVal[i * 2 + 1] = (p & 0xF) > 9 ? (p - 9) & 0xF | a : p & 0xF | '0';
            }
            return retVal;
        }

        template<typename T>
        std::string buildMacCmdString(MacCommand cmd, T value = 0, GetSet prop = NONE) {
            std::string retVal = "mac ";
            if (prop == SET) {
                retVal.append("set ");
            } else if (prop == GET) {
                retVal.append("get ");
            }
            switch (cmd) {
                case PAUSE:
                    retVal.append("pause");
                    break;
                default:
                    retVal.append("");
                    //TODO implement all mac commands
            }
            retVal.append("\r\n");
            return retVal;
        }

        template<typename T>
        std::string buildRadioCmdString(RadioCommand cmd, T value = 0, GetSet prop = SET) {
            //TODO implement get set command builder now it is just setter
            //TODO make it work with values of the string type
            std::string retVal = "radio ";
            if (prop == SET) {
                retVal.append("set ");
            } else if (prop == GET) {
                retVal.append("get ");
            }
            switch (cmd) {
                case MOD:
                    switch (value) {
                        case FSK:
                            retVal.append("mod fsk");
                            break;
                        default:
                            retVal.append("mod lora");
                    }
                    break;
                case FREQ:
                    retVal.append("freq ").append(std::to_string(value));
                    break;
                case PWR:
                    retVal.append("pwr ").append(std::to_string(value));
                    break;
                case SF:
                    retVal.append("sf sf").append(std::to_string(static_cast<int >(value)));
                    break;
                case AFCBW:
                    retVal.append("afcbw ").append(bandWidthToString(static_cast<BandWidth>(value)));
                    break;
                case RXBW:
                    retVal.append("rxbw ").append(bandWidthToString(static_cast<BandWidth>(value)));
                    break;
                case FSKBITRATE:
                    //TODO implement when FSK is supported
                    break;
                case FDEV:
                    retVal.append("fdev ").append(std::to_string(static_cast<int>(value)));
                    break;
                case PRLEN:
                    retVal.append("prlen ").append(std::to_string(static_cast<int>(value)));
                    break;
                case CRC:
                    if (value) {
                        retVal.append("crc on");
                    } else {
                        retVal.append("crc off");
                    }
                    break;
                case CR:
                    retVal.append("cr ").append(codingRateToString(static_cast<CodingRate>(value)));
                    break;
                case WDT:
                    retVal.append("wdt ").append(std::to_string(value));
                    break;
                case SYNC:
                    retVal.append("sync ").append(std::to_string(value));
                    break;
                case BW:
                    retVal.append("bw ").append(std::to_string(static_cast<int>(value)));
                    break;
                case rRX:
                    retVal.append("rx 0");
                    break;
            }
            retVal.append("\r\n");
            return retVal;
        }

        std::string buildRadioCmdString(RadioCommand cmd, std::string value, GetSet prop = SET) {
            std::string retVal = "radio ";
            if (prop == SET) {
                retVal.append("set ");
            } else if (prop == GET) {
                retVal.append("get ");
            }
            if (cmd == rTX) {
                std::string msg = "";
                stringToHex(value, msg);
                retVal.append("tx ").append(msg);
            }
            retVal.append("\r\n");
            return retVal;
        }

        unsigned long calculateDownTime(unsigned int payload);

        void write(const std::string &value);

        void rx();

        void macpause();

        bool proceed_;
        bool ignoreWarn_;
        bool listen_;
        protocol::Serial serial_;//!< SerialPort for UART communication with the chip
        unsigned int baudrate_; //<! BaudRate for UART communication with the chip
        ModulationMode mod_; //!< Modulation mode
        unsigned long freq_; //!< Frequency between 433050000..4347900000 or 863000000...870000000
        int8_t pwr_; //!< Power of transmission between -3...15
        SpreadingFactor sf_; //!< Spreading factor of the signal
        BandWidth afcbw_; //!< Automatic frequency correction in kHz
        BandWidth rxbw_; //!< Signal bandwidth in kHz
        uint fskBitRate_; //!< FSK bitrate between 1...300000
        uint fdev_; //!< Frequency deviation between 0...200000
        uint prlen_; //!< Preamble length between 0...65535
        bool crc_; //!< CRC Header on or off
        CodingRate cr_; //!< The coding rate
        unsigned long wdt_; //!< WatchDog 0...4294967295. Set to 0 to disable
        unsigned int sync_; //!< Sync word
        RadioBandWidth bw_; //!< RadioBandWidth in kHz
        bool sendAllowed_;
    };


}
