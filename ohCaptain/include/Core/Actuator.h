//
// Created by peer23peer on 7/23/16.
//

#pragma once

#include <boost/shared_ptr.hpp>

#include <string>
#include <vector>

#include "Controller.h"
#include "Exception.h"

namespace oCpt {

    class iActuator {
    public:
        typedef boost::shared_ptr<iActuator> ptr;

        iActuator();

        virtual ~iActuator();

        virtual void setActuator() = 0;

        virtual void run() = 0;

        virtual void stop() = 0;
    };

    class Actuator : public iActuator {
    public:
        Actuator();

        virtual ~Actuator() override;

        virtual void setActuator() override;

        virtual void run() override;

        virtual void stop() override;

    };
}
