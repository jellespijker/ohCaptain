//
// Created by peer23peer on 7/23/16.
//

#include "../../include/Vessels/Meetcatamaran.h"
#include "../../include/Controllers/BeagleboneBlack.h"
#include "../../include/Sensors/PT100.h"

using namespace oCpt::components;

namespace oCpt {
    namespace vessels {
        Meetcatamaran::Meetcatamaran() {
            controller_ = controller::BBB::ptr(new controller::BBB(world_));
            sensors::PT100::ptr tempSensor1(new sensors::PT100(controller_, world_ ,"tempSensor1", 0, 0));
            tempSensor1->setTimer(boost::posix_time::milliseconds(5000));
            boatswain_->registerSensor(tempSensor1);
            sensors_.push_back(tempSensor1);
            sensors::PT100::ptr tempSensor2(new sensors::PT100(controller_, world_, "tempSensor2", 6, 0));
            tempSensor2->setTimer(boost::posix_time::milliseconds(5500));
            boatswain_->registerSensor(tempSensor2);
            sensors_.push_back(tempSensor2);
        }

        Meetcatamaran::~Meetcatamaran() {

        }
    }
}