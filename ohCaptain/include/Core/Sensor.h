//
// Created by peer23peer on 7/15/16.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <boost/chrono.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <string>
#include <vector>

#include "Controller.h"
#include "Exception.h"

namespace oCpt {

    class iSensor {
    public:
        typedef boost::shared_ptr<iSensor> ptr;
        typedef boost::signals2::signal<void()> signal_t;

        struct State {
            union {
                char _char_t;
                unsigned char _uchar_t;
                short _short_t;
                unsigned short _ushort_;
                int _int_t;
                unsigned int _uint_t;
                long _long_t;
                unsigned long _ulong_t;
                long long _longlong_t;
                unsigned long long _ulonglong_t;
                float _float_t;
                double _double_t;
                long double _longdouble_t;
                bool _bool_t;
            } Value;
            boost::chrono::time_point<boost::chrono::steady_clock> Stamp;
        };

        iSensor(iController::ptr controller, World::ptr world ,std::string id, std::string typeOfSensor = "");

        virtual ~iSensor();

        virtual void updateSensor() = 0;

        virtual void run() = 0;

        virtual void stop() = 0;

        virtual bool operator==(iSensor::ptr rhs);

        const boost::posix_time::milliseconds &getTimer() const;

        void setTimer(const boost::posix_time::milliseconds &timer);

        const signal_t &getSig_() const;

        const State &getState() const;

    protected:
        std::string id_;
        std::string typeOfSensor_;
        iController::ptr controller_;
        World::ptr world_;
        boost::posix_time::milliseconds timer_;
        signal_t sig_;
        State state_;
    };

    class Sensor : public iSensor {
    public:
        Sensor(iController::ptr controller, World::ptr world, std::string id, std::string typeOfSensor);

        virtual ~Sensor() override;

        virtual void updateSensor() override;

        virtual void run() override;

        virtual void stop() override;


    };


}
