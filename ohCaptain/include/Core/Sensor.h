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

#define CAST(x, t) boost::any_cast<t::ReturnValue_t>(x) /*<! CAST the return value of a generic boost::any object, which can change for each sensor to a the proper return value. where the first parameter is the getState().Value and the second is the Sensor Class.
 */

namespace oCpt {

    /*!
     * Each sensor that is used should adhere to the sensor interface. A sensor consits of an connection to a controller, such as a ARM device and the world. The sensor needs to be initiated with the construct, where afterwards the init fucntion is called. The sensor should then be registerd by the Boatswain, using Boatswain::registerSensor(). This ensures that the boatswain can run the sensors. Some sensors are automatically update, whilst other need a manual action, such it is common practice to call the iSensor::updateSensor(). Once the value is update, a new Boost::Signal2 is fired, which allow for the main function to obtain the State of the sensor. Via iSensor::getState(). Since the return value of a sensor can vary, it is important to note that the final sensor should include a typedef with the return type named ReturnValue_t. After a sensor update is given or a signal is recieved, the return value can be CAST using the macro CAST(x,t)
     */
    class iSensor {
    public:
        typedef boost::shared_ptr<iSensor> ptr; //<! Boost shared_ptr for a sensor
        typedef boost::signals2::signal<void()> signal_t; //<! Signal type which is emited when a state is changed
        typedef boost::any generic_t; //<! Generic returnvalue, which allows for type-safe Polymorphic return calls

        struct State {
            generic_t Value; //<! a generic return value
            World::Time::timepoint_t Stamp; //<! A Epoch when this value was obained
        }; //<! State of a sensor at a certain time

        /*!
         * Constructor of iSensor
         * @param controller a shared_ptr of the controller where the sensor is hooked to
         * @param world a shared_ptr of the world in which the vessel operates
         * @param id a identifying name of the sensor
         * @param typeOfSensor a identifying category for the sensor
         */
        iSensor(iController::ptr controller, World::ptr world, std::string id, std::string typeOfSensor = "");

        /*!
         * Deconstructor of the sensor
         */
        virtual ~iSensor();

        /*!
         * pure virtual function for the updating of a sensor
         */
        virtual void updateSensor() = 0;

        /*!
         * pure virtual function for running of the sensor
         */
        virtual void run() = 0;

        /*!
         * pure virtual function for stopping the sensor
         */
        virtual void stop() = 0;

        /*!
         * pure virtual function for initializing the sensor
         */
        virtual void init() = 0;

        /*!
         * pure virtual function for registering the Input Output service
         * @param ioservice teh Input Output service used by Boost ASIO
         */
        virtual void setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice) = 0;

        /*!
         * Equal operator determining if this sensor is equal with the pointer
         * @param rhs shared_ptr with the other sensor
         * @return returns either true or false
         */
        virtual bool operator==(iSensor::ptr rhs);

        /*!
         * Get the number of milliseconds when this sensor should be updated
         * @return returns a boost::posix_time::milliseconds type
         */
        const boost::posix_time::milliseconds &getTimer() const;

        /*!
         * set the number of milliseconds when this sensor should be updated
         * @param timer  the number of milliseconds as an boost::posix_time::milliseconds type
         */
        void setTimer(const boost::posix_time::milliseconds &timer);

        /*!
         * get the signal that is to be fired when the state is updated
         * @return the signal_t
         */
        signal_t &getSig();

        /*!
         * gets the last State of the sensor
         * @return the State object. Remember to CAST the value like such <sensorClass>::ReturnValue_t ret = CAST(<sensorname>->getState().Value, <sensorClass>);
         */
        const State &getState() const;

    protected:
        std::string id_; //<! the string identifier of the sensor
    public:
        /*!
         * get the current ID
         * @return returns the ID as string
         */
        const std::string &getID() const;

        /*!
         * sets the ID of the sensor
         * @param id identifying string
         */
        void setID(const std::string &id);

        /*!
         * get the type of sensor
         * @return  category identifying string
         */
        const std::string &getTypeOfSensor() const;

        /*!
         * sets the category of the sensor, suchs as GPS, temperature
         * @param typeOfSensor category identifying string
         */
        void setTypeOfSensor(const std::string &typeOfSensor);

    protected:
        std::string typeOfSensor_; //<! Type of sensor
        iController::ptr controller_; //< shared_ptr to the controller
        World::ptr world_; //<! shared_ptr to the world
        boost::posix_time::milliseconds timer_; //<! milliseconds interval when an update should take place
        signal_t sig_;
        State state_;
        bool sensorRunning_ ;
        boost::shared_ptr<boost::asio::io_service> ioservice_;
    };

    /*!
     * Implementation of the iSensor interface
     */
    class Sensor : public iSensor {
    public:
        /*!
         * Constructor of Sensor
         * @param controller a shared_ptr of the controller where the sensor is hooked to
         * @param world a shared_ptr of the world in which the vessel operates
         * @param id a identifying name of the sensor
         * @param typeOfSensor a identifying category for the sensor
         */
        Sensor(iController::ptr controller, World::ptr world, std::string id, std::string typeOfSensor);

        /*!
         * Deconstructor of the Sensor class
         */
        virtual ~Sensor() override;

        /*!
         * virtual function which performs a sensor update, obtaining a  new value and sending a signal afterwards
         */
        virtual void updateSensor() override;

        /*!
         * virtual function starting the run service for the IO
         */
        virtual void run() override;

        /*!
         * virtual function stopping the run
         */
        virtual void stop() override;

        /*!
         * Initialize the sensor
         */
        virtual void init() override;

        /*!
         * Setting the used Asynchronous Input Output service
         * @param ioservice ASIO IO service, which handles the async calls from multiple sensors
         */
        virtual void setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice) override;


    };
}
