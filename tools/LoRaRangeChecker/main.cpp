#include <stdio.h>
#include <iostream>

#include "../../ohCaptain/include/Controllers/BeagleboneBlack.h"
#include "../../ohCaptain/include/Communication/LoRa_RN2483.h"
#include "../../ohCaptain/include/Core/Vessel.h"
#include "../../ohCaptain/include/Core/Controller.h"

using namespace oCpt::components;

namespace oCpt {
    namespace vessels {
        class LoRaVessel : public oCpt::Vessel {
        public:
            void sendCmd(std::string cmd, comm::LoRa_RN2483::ptr rn2483) {
                std::cout << cmd << std::endl;
                oCpt::iComm::Message msg(cmd, world_->now());
                rn2483->sendMessage(msg);
                sleep(1);
            }

            LoRaVessel() {
                std::cout << "Setting up controller" << std::endl;
                controller_ = controller::BBB::ptr(new controller::BBB(world_));

                std::cout << "Setting up LoRa RN2483 chip" << std::endl;
                comm::LoRa_RN2483::ptr rn2483(new comm::LoRa_RN2483("rn2483", "/dev/ttyACM1"));
                boatswain_->registerComm(rn2483);
                comm_.push_back(rn2483);
                rn2483->msgRecievedSig.connect([&] {
                    std::cout << rn2483->readFiFoMsg()->Payload << std::endl;
                });

                std::cout << "Setting up GPS" << std::endl;

                std::cout << "Setting LoRa in node 2 node configuration" << std::endl;
                rn2483->initialize();
                boatswain_->run();
                for(;;)
//                sendCmd("sys reset", rn2483);
//                sendCmd("radio set mod lora", rn2483);
//                sendCmd("radio set freq 868000000", rn2483);
//                sendCmd("radio set pwr 14", rn2483);
//                sendCmd("radio set sf sf12", rn2483);
//                sendCmd("radio set afcbw 125", rn2483);
//                sendCmd("radio set rxbw 250", rn2483);
//                sendCmd("radio set fdev 5000", rn2483);
//                sendCmd("radio set prlen 8", rn2483);
//                sendCmd("radio set crc on", rn2483);
//                sendCmd("radio set cr 4/8", rn2483);
//                sendCmd("radio set wdt 0", rn2483);
//                sendCmd("radio set sync 12", rn2483);
//                sendCmd("radio set bw 250", rn2483);
//                sendCmd("mac pause", rn2483);
//                sendCmd("radio tx FF", rn2483);
                boatswain_->stop();
            }

            virtual ~LoRaVessel() {

            }

        private:

        };
    }
}

int main(int argc, const char *argv[]) {
    std::cout << "Starting LoRaRange finder" << std::endl;
    oCpt::vessels::LoRaVessel lv;
}
