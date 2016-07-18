//
// Created by peer23peer on 7/16/16.
//

#include "../../include/Core/Vessel.h"

namespace oCpt {

    iVessel::iVessel() {}

    iVessel::~iVessel() {}

    Vessel::Vessel() {
        _world = World::ptr( new World() );
    }

    Vessel::Vessel(iController::ptr controller) {
        _controller = controller;
        _world = World::ptr( new World() );
    }

    Vessel::~Vessel() {}

}