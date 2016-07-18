//
// Created by peer23peer on 7/15/16.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <string>
#include <vector>

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

        class adc : public userspace  {
        public:
            typedef boost::shared_ptr<adc> ptr;

            adc(uint8_t id, uint8_t device, std::string modName = "");
            virtual ~adc();

            uint16_t &getValue();
            bool operator==(const adc &rhs);
            bool compare(const uint8_t &id, const uint8_t &device = 0);
        private:
            uint8_t _id = 0;
            uint8_t _device = 0;
            std::string _path = "";
            uint16_t _value = 0;
        };
    }

    class iController {
    public:
        typedef boost::shared_ptr<iController> ptr;

        iController();
        virtual ~iController();

        virtual std::vector<protocol::adc::ptr> *getAdcVector() = 0;
        virtual protocol::adc::ptr getADC(uint8_t id, uint8_t device) = 0;
    protected:
        std::vector<protocol::adc::ptr> _adcVector;
    };

    class ARM : public iController {
    public:
        ARM();
        virtual ~ARM();
        virtual std::vector<protocol::adc::ptr> *getAdcVector();
        virtual protocol::adc::ptr getADC(uint8_t id, uint8_t device);
    };

    class BBB : public ARM {
    public:
        //typedef boost::shared_ptr<iController> ptr;
        BBB();
        virtual ~BBB();
    };



 }
