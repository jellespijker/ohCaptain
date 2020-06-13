//
// Created by peer23peer on 7/23/16.
//

#include "../../include/Vessels/Meetcatamaran.h"
#include "../../include/Controllers/BeagleboneBlack.h"
#include "../../include/Sensors/PT100.h"
#include "../../include/Communication/LoRa_RN2483.h"

#include <boost/bind.hpp>
#include <iostream>

using namespace oCpt::components;

namespace oCpt {
    namespace vessels {
        Meetcatamaran::Meetcatamaran() {
            controller_ = controller::BBB::ptr(new controller::BBB(world_));

            sensors::PT100::ptr tempSensor1(new sensors::PT100(controller_, world_, "tempSensor1", 0, 0));
            tempSensor1->setTimer(boost::posix_time::milliseconds(5000));
            boatswain_->registerSensor(tempSensor1);
            sensors_.push_back(tempSensor1);

            comm::LoRa_RN2483::ptr rn2483(new comm::LoRa_RN2483("rn2483", "/dev/ttyACM0"));
            boatswain_->registerComm(rn2483);
            comm_.push_back(rn2483);

        }

        Meetcatamaran::~Meetcatamaran() {
            boatswain_->stop();
        }

    }
}