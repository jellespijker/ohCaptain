//
// Created by peer23peer on 7/23/16.
//

#include "../../include/Controllers/BeagleboneBlack.h"


namespace oCpt {
    namespace components {
        namespace controller {
            BBB::BBB(World::ptr world)
                    : ARM(world) {
                // Init IIO device 0 port 0..6 load
                for (uint8_t i = 0; i < 7; i++) {
                    protocol::adc::ptr adc_in(new protocol::adc(i, 0, "ti_am335x_adc"));
                    adcVector_.push_back(adc_in);
                }
            }

            BBB::~BBB() {}
        }
    }
}