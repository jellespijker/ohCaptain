//
// Created by peer23peer on 7/15/16.
//

#include "../../include/Core/Controller.h"
#include "../../include/Core/Exception.h"

#include <sys/stat.h>
#include <fstream>
#include <sstream>

#include <boost/make_shared.hpp>

namespace oCpt {

    namespace protocol {

        userspace::userspace() {}

        userspace::~userspace() {}

        bool userspace::modLoaded(std::string modName) {
            if (modName.compare("") == 0) { return true; }
            std::ifstream fs("/proc/modules");
            std::string line;
            while (std::getline(fs, line)) {
                if (line.find(modName, 0) != std::string::npos) {
                    fs.close();
                    return true;
                }
            }
            fs.close();
            return false;
        }

        bool userspace::fileExist(std::string fileName) {
            struct stat buffer;
            return (stat (fileName.c_str(), &buffer) == 0);
        }

        bool userspace::dtboLoaded(std::string dtboName) {
            return  true;
        }

        adc::adc(uint8_t id, uint8_t device, std::string modName) {
            _device = device;
            _id = id;
            std::stringstream ss;
            ss << "/sys/bus/iio/devices/iio:device"  << std::to_string(_device) << "/in_voltage" << std::to_string(_id) << "_raw";
            _path = ss.str();
            if (!modLoaded(modName)) {
                throw oCptException("Exception! Module not loaded", 0);
            }
            if (!fileExist(_path)) {
                throw oCptException("Exception! Userspacefile doesn't exist", 1);
            }
        }

        adc::~adc() {}

        uint16_t  &adc::getValue() {
            std::ifstream fs;
            fs.open(_path.c_str());
            fs >> _value;
            fs.close();
            return _value;
        }

        bool adc::operator==(const adc &rhs) {
            return (rhs._path.compare(_path) == 0);
        }

        bool adc::compare(const uint8_t &id, const uint8_t &device) {
            return (_id == id && _device == device);
        }
    }

    iController::iController() {}

    iController::~iController() {}

    ARM::ARM() {
    }

    ARM::~ARM() {}

    protocol::adc::ptr ARM::getADC(uint8_t id, uint8_t device) {
        std::for_each(_adcVector.begin(), _adcVector.end(),[&](protocol::adc::ptr &A){
           if (A->compare(id, device)) { return A; }
        });
        return protocol::adc::ptr();
    }

    std::vector<protocol::adc::ptr> *ARM::getAdcVector() {
        return &_adcVector;
    }

    BBB::BBB() {
        // Init IIO device 0 port 0..6 load
        for (uint8_t i = 0; i < 7; i++) {
            protocol::adc::ptr adc_in(new protocol::adc(i, 0, "ti_am335x_adc"));
            _adcVector.push_back(adc_in);
        }
    }

    BBB::~BBB() {}

}