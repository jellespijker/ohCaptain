//
// Created by peer23peer on 7/23/16.
//

#pragma once

#include <boost/shared_ptr.hpp>

#include "World.h"

namespace oCpt {
    class iCaptain {
    public:
        typedef boost::shared_ptr<iCaptain> ptr;

        iCaptain(World::ptr world);

        virtual ~iCaptain();

        virtual void run() = 0;

        virtual void stop() = 0;

        virtual void initialize() = 0;

    protected:
        World::ptr world_;
    };

    class Captain : public iCaptain {
    public:
        Captain(World::ptr world);

        virtual ~Captain() override;

        virtual void run() override;

        virtual void stop() override;

        virtual void initialize() override;

    };
}
