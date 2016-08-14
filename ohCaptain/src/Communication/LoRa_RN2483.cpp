//
// Created by peer23peer on 8/12/16.
//

#include "../../include/Core/Communication.h"
#include "../../include/Communication/LoRa_RN2483.h"

namespace oCpt {
    namespace components {
        namespace comm {

            LoRa_RN2483::LoRa_RN2483(const std::string &id, const std::string &device, World::ptr world,
                                     iController::io_t ioservice)
                    : LoRa(id, device, world, ioservice) {

            }

            LoRa_RN2483::~LoRa_RN2483() {

            }
        }
    }
}

