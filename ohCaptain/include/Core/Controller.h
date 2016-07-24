//
// Created by peer23peer on 7/15/16.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <string>
#include <vector>

#include "World.h"

namespace oCpt {

    namespace protocol {
        class userspace {
        public:
            userspace();

            virtual ~userspace();

        protected:
            bool modLoaded(std::string modName);

            bool fileExist(std::string fileName);

            bool dtboLoaded(std::string dtboName);
        };

        class adc : public userspace {
        public:
            typedef boost::shared_ptr<adc> ptr;

            adc(uint8_t id, uint8_t device, std::string modName = "");

            virtual ~adc();

            uint16_t &getValue();

            bool operator==(const adc &rhs);

            bool compare(const uint8_t &id, const uint8_t &device = 0);

        private:
            uint8_t id_ = 0;
            uint8_t device_ = 0;
            std::string path_ = "";
            uint16_t value_ = 0;
        };
    }

    class iController {
    public:
        typedef boost::shared_ptr<iController> ptr;

        iController(World::ptr world);

        virtual ~iController();

        virtual std::vector<protocol::adc::ptr> *getAdcVector() = 0;

        virtual protocol::adc::ptr getADC(uint8_t id, uint8_t device) = 0;

    protected:
        std::vector<protocol::adc::ptr> adcVector_;
        World::ptr world_;
    };

    class ARM : public iController {
    public:
        ARM(World::ptr world);

        virtual ~ARM();

        virtual std::vector<protocol::adc::ptr> *getAdcVector();

        virtual protocol::adc::ptr getADC(uint8_t id, uint8_t device);
    };
}
