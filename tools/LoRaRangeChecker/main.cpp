#include <stdio.h>
#include <iostream>

#include "../../ohCaptain/include/Controllers/BeagleboneBlack.h"
#include "../../ohCaptain/include/Communication/LoRa_RN2483.h"
#include "../../ohCaptain/include/Core/Vessel.h"
#include "../../ohCaptain/include/Sensors/Gps.h"
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

            void sendGPS(std::string loc) {
                comm::LoRa_RN2483::Message msg(loc, world_->now());
                comm_[0]->sendMessage(msg);
            }

            LoRaVessel() {
                std::cout << "Setting up controller" << std::endl;
                controller_ = controller::BBB::ptr(new controller::BBB(world_));

                std::cout << "Setting up LoRa RN2483 chip" << std::endl;
                comm::LoRa_RN2483::ptr rn2483(new comm::LoRa_RN2483("rn2483", "/dev/ttyACM0"));
                boatswain_->registerComm(rn2483);
                comm_.push_back(rn2483);
                rn2483->msgRecievedSig.connect([&] {
                    std::cout << rn2483->readFiFoMsg()->Payload << std::endl;
                });

                std::cout << "Setting up GPS" << std::endl;
                sensors::Gps::ptr gps(new sensors::Gps(controller_,world_,"gps","/dev/gps",115200));
                boatswain_->registerSensor(gps);
                sensors_.push_back(gps);
                std::cout << "Setting up GPS relay function" << std::endl;
                gps->getSig().connect([&]{
                    sensors::Gps::ReturnValue_t ret = CAST(gps->getState().Value, sensors::Gps);
                    comm::LoRa_RN2483::Message msg(ret.toString(), gps->getState().Stamp);
                    rn2483->sendMessage(msg);
                    std::cout << "Sending GPS coordinates : " << ret.toString() << std::endl;
                });

                std::cout << "Setting LoRa in node 2 node configuration" << std::endl;
                rn2483->initialize();

                boatswain_->run();
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
