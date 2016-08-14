//
// Created by peer23peer on 8/12/16.
//
#pragma once

#include "../Core/Communication.h"

//Yes point to point is possible . Ive tried this myself. use this initialisation on both TX and RX
//"radio set mod lora"
//"radio set freq 868000000"
//"radio set pwr 14"
//"radio set sf sf12"
//"radio set afcbw 125"
//"radio set rxbw 250"
//"radio set fdev 5000"
//"radio set prlen 8"
//"radio set crc on"
//"radio set cr 4/8"
//"radio set wdt 0"
//"radio set sync 12"
//"radio set bw 250"
//
//on the receiver side use these commands:
//"mac pause"
//radio rx 0"
//
//on the transmitter side use these commands to send a packet eg FFh
//"mac pause"
//"radio tx FF"

namespace oCpt {
    namespace components {
        namespace comm {
            class LoRa_RN2483 : public LoRa {
            public:

                LoRa_RN2483(const std::string &id, const std::string &device,
                            World::ptr world = World::ptr(new World()),
                            iController::io_t ioservice = iController::io_t(new boost::asio::io_service()));

                virtual ~LoRa_RN2483();
            };
        }
    }
}
