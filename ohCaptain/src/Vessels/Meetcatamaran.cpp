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
            controller_ = controller::BBB::ptr( new controller::BBB(world_) );
            sensors::PT100::ptr tempSensor( new sensors::PT100(controller_, "tempSensor", 0, 0) );
            tempSensor->setTimer(boost::posix_time::milliseconds(10000));
            boatswain_->registerSensor(tempSensor);
            sensors_.push_back(tempSensor);
            auto test = tempSensor->getState().Value;
        }

        Meetcatamaran::~Meetcatamaran() {

        }
    }
}