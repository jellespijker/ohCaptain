//
// Created by peer23peer on 7/23/16.
//

#include "../../include/Vessels/Meetcatamaran.h"
#include "../../include/Controllers/BeagleboneBlack.h"
#include "../../include/Sensors/PT100.h"
#include "../../include/Sensors/NavilockNLX02.h"

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

            sensors::NavilockNLX02::ptr locSensor(
                    new sensors::NavilockNLX02(controller_, world_, "locSensor", "/dev/ttyUSB0", 4800));
            boatswain_->registerSensor(locSensor);
            sensors_.push_back(locSensor);
//
//            sensors::PT100::ptr tempSensor2(new sensors::PT100(controller_, world_, "tempSensor2", 6, 0));
//            tempSensor2->setTimer(boost::posix_time::milliseconds(10000));
//            boatswain_->registerSensor(tempSensor2);
//            sensors_.push_back(tempSensor2);
        }

        Meetcatamaran::~Meetcatamaran() {
            boatswain_->stop();
        }

    }
}