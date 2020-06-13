//
// Created by peer23peer on 7/21/16.
//

#pragma once

#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include "Controller.h"
#include "Sensor.h"
#include "Actuator.h"
#include "Communication.h"

namespace oCpt {

    /*!
     * The Boatswain performs all the labours tasks, suchs updateing and interpretting sensor readings, setting actuators according to the Captain wishes, updating the state representation of the vessel in the World. Each Boatswain runs on its own thread. It is possible for a vessel to have multiple Boatswains, responsible for multiple tasks, such as communication, localization, steering. Each Boatswain has to adhere to the iBoatswain interface.
     */
    class iBoatswain : public boost::enable_shared_from_this<iBoatswain> {
    public:
        typedef boost::shared_ptr<iBoatswain> ptr; //<! Boost shared_ptr for a Boatswain
        typedef boost::shared_ptr<boost::asio::deadline_timer> timerPtr; //<! Boost shared_ptr Dealine timer //TODO check if this is still needed could be left over from old construct

        /*!
         * Constructor for a iBoatswain
         * @param controller a shared_ptr to the controller with which teh Boatswain interacts
         */
        iBoatswain(iController::ptr controller);

        /*!
         * Deconstructor for the iBoatswain
         */
        virtual ~iBoatswain();

        /*!
         * pure virtual function for running the boatswain and his registered sensors
         */
        virtual void run() = 0;

        /*!
         * pure virtual function for stopping the run task
         */
        virtual void stop() = 0;

        /*!
         * pure virtual function of initialzing the Boatswain
         */
        virtual void initialize() = 0;

        /*!
         * Pure virtual function for registering a new sensor with the Boatswain
         * @param sensor a shared_ptr to a Sensor which need to maintained by the Boatswain
         */
        virtual void registerSensor(iSensor::ptr sensor) = 0;

        /*!
         * Pure virtual function for registering a new actuator with the Boatswain
         * @param actuator a shared_ptr to an Actuator which need to be maintained by the Boatswain
         */
        virtual void registerActuator(iActuator::ptr actuator) = 0;

        /*!
         * Pure virtual function for registering a new communication device which
         * @param comm
         */
        virtual void registerComm(iComm::ptr comm) = 0;

        /*!
         * get if the thread is stopped
         * @return returns if the thread should stop
         */
        const boost::shared_ptr<bool> &getStopThread() const;

        /*!
         * set the value of the stopthread
         * @param stopThread //<! a shared_ptr to the boolean
         */
        void setStopThread(const boost::shared_ptr<bool> &stopThread);

        /*!
         * get the used Input Output service
         * @return a shared_ptr to the ASIO io service
         */
        boost::shared_ptr<boost::asio::io_service> &getIOservice();

    protected:
        boost::shared_ptr<boost::asio::io_service> ioservice_; //<! a shared_ptr to the used io service;
        iController::ptr controller_; //<! a shared_ptr to the controller
        std::vector<timerPtr> timers_; //<! a vector consisting of multiple dead_line timers
        std::vector<iSensor::ptr> timerSensors_; //<! a vector of Sensor which need to be timed
        std::vector<iSensor::ptr> manualSensors_; //<! a vector of Sensor which need to be manually updated
        boost::shared_ptr<bool> stopThread_; //<! a shared_ptr to a boolean, which is connected with other threads that need to be stopped/started simultaneous
        boost::shared_ptr<bool> localStopThread_;

        /*!
         * Pure virtual function for resetting the timer
         * @param sensor
         */
        virtual void resetTimer(iSensor::ptr sensor) = 0;
    };

    /*!
     * The Boatswain performs all the labours tasks, suchs updateing and interpretting sensor readings, setting actuators according to the Captain wishes, updating the state representation of the vessel in the World. Each Boatswain runs on its own thread. It is possible for a vessel to have multiple Boatswains, responsible for multiple tasks, such as communication, localization, steering. Each Boatswain has to adhere to the iBoatswain interface.
     */
    class Boatswain : public iBoatswain {
    public:
        /*!
         * The constructor for a Boatswain
         * @param controller a shared_ptr to the controller with which teh Boatswain interacts
         */
        Boatswain(iController::ptr controller);

        /*!
         * Deconstructor
         */
        virtual ~Boatswain() override;

        /*!
         * Make the Boatswain work and execute the actuators, sensors and communications
         */
        virtual void run() override;

        /*!
         * Stop the execution of the tasks
         */
        virtual void stop() override;

        /*!
         * Initialize the Boatswain
         */
        virtual void initialize() override;

        /*!
         * Register a new Sensor with the Boatswain. If the Timer for the Sensor is set to a value greater the 0, the Sensor is registered with the timerSensors_ and a timer is set. Otherwise the Sensor is registered as a manualSensors_
         * @param sensor a shared_ptr to a Sensor
         */
        virtual void registerSensor(iSensor::ptr sensor) override;

        /*!
         * Register a new Actuator with the Boatswain
         * @param actuator a shared_ptr to an Actuator
         */
        virtual void registerActuator(iActuator::ptr actuator) override;

        /*!
         * Register a new iComm device by setting a shared IO service
         * @param comm
         */
        virtual void registerComm(iComm::ptr comm) override;

    protected:
        /*!
         * reset the timer of a Sensor
         * @param sensor a shared_ptr to the Sensor
         */
        void resetTimer(iSensor::ptr sensor) override;

    };
}
