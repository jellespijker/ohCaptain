//
// Created by peer23peer on 7/15/16.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>

#include <string>
#include <vector>

namespace oCpt {

    class iSensor {
    public:

        class iState {
        public:
            typedef boost::shared_ptr<iState> ptr;
            iState(const std::string &symbol, const auto &value);
            virtual ~iState();

            auto &getValue();
            std::string &getSymbol();
        protected:
            std::string _symbol;
            auto _value;
        };

        class Parameter : public iState {
        public:
            Parameter(const std::string &symbol, const auto &value);
            ~Parameter();

            auto &getValue();
            std::string &getSymbol();
        };

        class SensorState : public iState {
        public:
            SensorState(const std::string &symbol, const auto &value);
            ~SensorState();

            auto &getValue();
            std::string &getSymbol();
        };

        typedef std::vector<Parameter::ptr> Parameters;
        typedef std::vector<SensorState::ptr> SensorStates;

        typedef boost::shared_ptr<iSensor> ptr;
        typedef boost::signals2::signal<auto ()> signal_t;

        iSensor(std::string id = "", Parameters &parameters);
        ~iSensor();

        std::string &getID();
        void setID(const std::string &id);

        std::string &getType();
        void setType(const std::string &typeName);

        virtual SensorState getState() = 0;
        virtual void Run() = 0;
        virtual void Stop() = 0;

        signal_t sig;

    protected:
        std::string _id = "";
        std::string _typeName = "";
        SensorState _state;
        bool _threadedAllowed = false;
    };

    class Sensor : public iSensor {
    public:
        Sensor(std::string id = "", Parameters &parameters);
        virtual ~Sensor();


    };

}
