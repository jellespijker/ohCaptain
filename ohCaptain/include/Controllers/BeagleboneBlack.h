//
// Created by peer23peer on 7/23/16.
//

#pragma once

#include "../Core/Controller.h"

#include <utility>

namespace oCpt {
    namespace components {
        namespace controller {
            class BBB : public ARM {
            public:
                //typedef boost::shared_ptr<iController> ptr;
                BBB(World::ptr world);

                virtual ~BBB();
            };

        }
    }
}
