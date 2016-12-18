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

    /*!
     * The interface for communication devices
     */
    class iComm {
    public:
        typedef boost::shared_ptr<iComm> ptr; //<! A typedef to the shared_ptr
        typedef boost::signals2::signal<void()> signal_t; //<! a typedef to the signal

        /*!
         * A Message struct. Each device should receive and send messages with this type, consisting of Payload in the format of a string and a time when it was send or received
         */
        struct Message {
            typedef boost::shared_ptr<Message> ptr; //<! A shared_ptr to a Message

            /*!
             * A Message constructor taking the payload and the time
             * @param payload A string containing the payload TODO make generic with template
             * @param stamp A time stamp, when the message was received, or is send
             */
            Message(const std::string &payload, const World::Time::timepoint_t &stamp)
                    : Payload(payload),
                      Stamp(stamp) {};

            /*!
             * The deconstructor
             */
            ~Message() {};
            std::string Payload; //<! The payload of the message TODO make generic
            World::Time::timepoint_t Stamp; //<! The time stamp
        };

        /*!
         * The constructor for the communication interface
         * @param id The ID of the communication device
         * @param device The device path eq. /dev/ttyS0
         * @param world  A shared_ptr to the world with a default to a newly created one
         * @param ioservice  A shared_ptr to an ASIO Input Output service with a newly created one as default
         */
        iComm(const std::string &id, const std::string &device, World::ptr world = World::ptr(new World()),
              iController::io_t ioservice = iController::io_t(new boost::asio::io_service()));

        /*!
         * The deconstructor
         */
        virtual ~iComm();

        /*!
         * a pure virtual function which runs the communication device
         */
        virtual void run() = 0;

        /*!
         * A pure virtual function which stops the communication device
         */
        virtual void stop() = 0;

        /*!
         * A pure virtual function which initializes the communication device
         */
        virtual void initialize() = 0;

        /*!
         * A pure virtual function which sends the message
         * @param msg the Message, consisting of a payload and a time stamp
         */
        virtual void sendMessage(Message msg) = 0;

        /*!
         * A pure virtual function with a shared_ptr to the first in queue received message, this function will hold the current thread
         * @return a shared_ptr pointing towards the queued Message
         */
        virtual Message::ptr recieveMessage() = 0;

        /*!
         * A pure virtual function which performs the polling for a new message on a seperate threads, so it won't block the current one, it needs to send a signal when the message is received
         */
        virtual void recieveAsyncMessage() = 0;

        /*!
         * Returns the ID of the communication device
         * @return a string with the ID
         */
        const std::string &getId() const;

        /*!
         * Set the ID of the communication device
         * @param id The ID of the communication device
         */
        void setId(const std::string &id);

        /*!
         * Get the type of communication device
         * @return a string with type of device eq. modem, serial, LoRa, WiFi
         */
        const std::string &getTypeOfComm() const;

        /*!
         * Set the type of communication device. eq. modem, serial, LoRa, WiFi
         * @param typeOfComm string representing the type of communication
         */
        void setTypeOfComm(const std::string &typeOfComm);

        /*!
         * Get a pointer to the first message in Queu
         * @return
         */
        Message::ptr readFiFoMsg();

        /*!
         * A que with received Message::ptr
         * @return a pointer to the Message que
         */
        std::deque<Message::ptr> *getMsgQueue();

        /*!
         * The ASIO Input Output service handling the messages
         * @param ioservice a shared_ptr to a IO service
         */
        void setIoservice(const iController::io_t &ioservice);

        /*!
         * A signal which is send when a new Message
         */
        signal_t msgRecievedSig;
    protected:
        std::string id_; //<! The communication device
        std::string typeOfComm_; //<! The type of communication device
        std::string device_; //<! The path to the device
        boost::posix_time::milliseconds timer_; //<! A timer value which can be set to send/receive at intervals TODO implment set/get
        std::deque<Message::ptr> msgQueue_; //<! A deque with the first recieved messages in the front and the last in the back
        iController::io_t ioservice_; //<! a shared_ptr to the ASIO IO service
        World::ptr world_; //<! a shere_ptr to the World
    };

    /*!
     * Communication class for the LoRa protocol. The current class is mostly based on node 2 node communication TODO rewrite sho it will allow mesh network communication. Most of the commands are taken from http://ww1.microchip.com/downloads/en/DeviceDoc/40001784B.pdf
     */
    class LoRa : public iComm {
    public:

        /*!
         * The modulation mode of the LoRa module
         */
        enum ModulationMode {
            LORA = 1, //<! Long Rang Low Power mode
            FSK = 2 //<! Frequency-shift keyring mode
        };

        /*!
         * The Spreading factor
         */
        enum SpreadingFactor {
            SF7 = 7,
            SF8 = 8,
            SF9 = 9,
            SF10 = 10,
            SF11 = 11,
            SF12 = 12
        };

        /*!
         * The bandwidth
         */
        enum BandWidth {
            BW250 = 1, //<! 250 kHz
            BW200 = 2, //<! 200 kHz
            BW166_7 = 3,//<! 166.7 kHz
            BW125 = 4, //<! 125 kHz
            BW100 = 5, //<! 100 kHz
            BW83_3 = 6, //<! 83.3 kHz
            BW62_5 = 7, //<! 62.5 kHz
            BW50 = 8, //<! 50 kHz
            BW41_7 = 9, //<! 41.7 kHz
            BW31_3 = 10, //<! 31.3 kHz
            BW25 = 11, //<! 25 kHz
            BW20_8 = 12, //<! 20.8 kHz
            BW15_6 = 13, //<! 15.6 kHz
            BW12_5 = 14, //<! 12.5 kHz
            BW10_4 = 15, //<! 10.4 kHz
            BW7_8 = 16, //<! 7.8 kHz
            BW6_3 = 17, //<! 6.3 kHz
            BW5_2 = 18, //<! 5.2 kHz
            BW3_9 = 19, //<! 3.9 kHz
            BW3_1 = 20, //<! 3.1 kHz
            BW2_6 = 21 //<! 2.6 kHz
        };

        /*!
         * The Coding rate of the signal
         */
        enum CodingRate {
            CR4_5 = 4, //<! CR 4/5
            CR4_6 = 3, //<! CR 4/6
            CR4_7 = 2, //<! CR 4/7
            CR4_8 = 1 //<! CR 4/8
        };

        /*!
         * The radio bandwidth
         */
        enum RadioBandWidth {
            RBW500 = 500, //<! 500 kHz
            RBW250 = 250, //<! 250 kHz
            RBW125 = 125 //<! 125 kHz
        };

        /*!
         * Perform a get or a set command or otherwise none
         */
        enum GetSet {
            GET,
            SET,
            NONE
        };

        /*!
         * Types of radio commands
         */
        enum RadioCommand {
            MOD, //<! modulation mode
            FREQ, //<! current operation requency for the radio
            PWR, //<! output power level used by the radio during transmission
            SF, //<! Spreading Factor to be used during transmission
            AFCBW, //<! automatic frequency correction bandwidth
            RXBW, //<! operational receive bandwidth
            FSKBITRATE, //<! frequency shift keyring bitrate
            FDEV, //<! frequency deviation allowed by the end device
            PRLEN, //<! preamble length used during transmissions
            CRC, //<! CRC header to be used
            CR, //<! Coding rate to be used
            WDT, //<! time-out limit for the radio watchdog timer
            SYNC, //<! sync word to be used
            BW, //<! value used for the bandwidth
            rRX,
            rTX
        };

        /*!
         * Type to control the MAC layer, currently only pause is used, because node 2 node communication doesn't use MAC
         */
        enum MacCommand {
            PAUSE,
            RESET,
            mTX,
            JOIN,
            SAVE,
            FORCEENABLE,
            RESUME
        };

        /*!
         * LoRa device constructor
         * @param id the ID of the device as an string
         * @param device the device path eq. /dev/ttyS0
         * @param world  shared_ptr to the World default = a newly created World
         * @param ioservice  shared_ptr to an IO service, default is a newly created IO service
         */
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

        /*!
         * Convert the value of a Type T to a hexidecimal string, which can be send to a LoRa device, such that it can be transmitted
         * @tparam T the type of value, to be converted
         * @param value the to be converted value
         * @param capital boolean indicating if the hexidecimal string should consist of capital letters
         * @return a string with the value as hexidecimal values
         */
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
        };

        /*!
         * A command string builder for MAC commands currently onlu PAUSE implemented
         * @tparam T the type of MAC command eq. MacCommand::PAUSE
         * @param cmd the MacCommand to be performed
         * @param value the Value to be send
         * @param prop Additional properties
         * @return a string which can be send to the LoRa module eq. "mac set pause"
         */
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
        };

        /*!
         * A command string builder for radio commands
         * @tparam T
         * @param cmd
         * @param value
         * @param prop
         * @return
         */
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
