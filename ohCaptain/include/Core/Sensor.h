//
// Created by peer23peer on 7/15/16.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <boost/chrono.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/any.hpp>

#include <string>
#include <vector>

#include "Controller.h"
#include "Exception.h"

#define CAST(x) boost::any_cast<Razor::ReturnValue_t>(x)

namespace oCpt {

    class iSensor {
    public:
        typedef boost::shared_ptr<iSensor> ptr;
        typedef boost::signals2::signal<void()> signal_t;
        typedef boost::any generic_t;

        struct State {
            generic_t Value;
            World::Time::timepoint_t Stamp;
        };

        iSensor(iController::ptr controller, World::ptr world, std::string id, std::string typeOfSensor = "");

        virtual ~iSensor();

        virtual void updateSensor() = 0;

        virtual void run() = 0;

        virtual void stop() = 0;

        virtual void init() = 0;

        virtual void setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice) = 0;

        virtual bool operator==(iSensor::ptr rhs);

        const boost::posix_time::milliseconds &getTimer() const;

        void setTimer(const boost::posix_time::milliseconds &timer);

        signal_t &getSig();

        const State &getState() const;

    protected:
        std::string id_;
    public:
        const std::string &getID() const;

        void setId_(const std::string &id_);

        const std::string &getTypeOfSensor() const;

        void setTypeOfSensor_(const std::string &typeOfSensor_);

    protected:
        std::string typeOfSensor_;
        iController::ptr controller_;
        World::ptr world_;
        boost::posix_time::milliseconds timer_;
        signal_t sig_;
        State state_;
        bool sensorRunning_ ;
    };

    class Sensor : public iSensor {
    public:
        Sensor(iController::ptr controller, World::ptr world, std::string id, std::string typeOfSensor);

        virtual ~Sensor() override;

        virtual void updateSensor() override;

        virtual void run() override;

        virtual void stop() override;

        virtual void init() override;

        virtual void setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice) override;


    };


}
