//
// Created by peer23peer on 7/15/16.
//

#include "../../include/Core/Controller.h"
#include "../../include/Core/Exception.h"

#include <sys/stat.h>
#include <fstream>

#include <boost/make_shared.hpp>

namespace oCpt {

    namespace protocol {

        userspace::userspace() {}

        userspace::~userspace() {}

        bool userspace::modLoaded(std::string modName) {
            if (modName.compare("") == 0) { return true; }
           // {
               // std::lock_guard<std::mutex> guard(usMutex);
                std::ifstream fs("/proc/modules");
                std::string line;
                while (std::getline(fs, line)) {
                    if (line.find(modName, 0) != std::string::npos) {
                        fs.close();
                        return true;
                    }
                }
                fs.close();
   //         }
            return false;
        }

        bool userspace::fileExist(std::string fileName) {
            struct stat buffer;
            return (stat(fileName.c_str(), &buffer) == 0);
        }

        bool userspace::dtboLoaded(std::string dtboName) {
            return true;
        }

        adc::adc(uint8_t id, uint8_t device, std::string modName) {
            device_ = device;
            id_ = id;
            std::stringstream ss;
            ss << "/sys/bus/iio/devices/iio:device" << std::to_string(device_) << "/in_voltage" << std::to_string(id_)
               << "_raw";
            path_ = ss.str();
            if (!modLoaded(modName)) {
                throw oCptException("Exception! Module not loaded", 0);
            }
            if (!fileExist(path_)) {
                throw oCptException("Exception! Userspacefile doesn't exist", 1);
            }
        }

        adc::~adc() {}

        uint16_t &adc::getValue() {
            //std::lock_guard<std::mutex> guard(usMutex); //TODO check if Mutex is needed for a read operation
            std::ifstream fs;
            fs.open(path_.c_str());
            fs >> value_;
            fs.close();
            return value_;
        }

        bool adc::operator==(const adc &rhs) {
            return (rhs.path_.compare(path_) == 0);
        }

        bool adc::compare(const uint8_t &id, const uint8_t &device) {
            return (id_ == id && device_ == device);
        }
    }

    iController::iController(World::ptr world)
            : world_(world) {}

    iController::~iController() {}

    ARM::ARM(World::ptr world)
            : iController(world) {}

    ARM::~ARM() {}

    protocol::adc::ptr ARM::getADC(uint8_t id, uint8_t device) {
        std::for_each(adcVector_.begin(), adcVector_.end(), [&](protocol::adc::ptr &A) {
            if (A->compare(id, device)) { return A; }
        });
        return protocol::adc::ptr();
    }

    std::vector<protocol::adc::ptr> *ARM::getAdcVector() {
        return &adcVector_;
    }
}