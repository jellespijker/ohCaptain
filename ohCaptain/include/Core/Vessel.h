//
// Created by peer23peer on 7/16/16.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <vector>
#include <thread>

#include "World.h"
#include "Controller.h"
#include "Captain.h"
#include "Boatswain.h"
#include "Actuator.h"
#include "Communication.h"

namespace oCpt {
    /*!
     * The interface for each vessel
     */
    class iVessel {
    public:
        typedef boost::shared_ptr<iVessel> ptr; //!< Boost shared_ptr to a vessel

        /*!
         * Constructor of the vessel interface
         * @return
         */
        iVessel();

        /*!
         * Constructor of the vessel interface
         * @param controller shared_ptr to the controller
         * @return
         */
        iVessel(iController::ptr controller);

        /*!
         * Deconstructor
         */
        virtual ~iVessel();

        /*!
         * Initialize the vessel
         */
        virtual void initialize() = 0;

        /*!
         * Run the vessel normal operations
         */
        virtual void run() = 0;

        /*!
         * Stop the vessel, everything except critical parts, which are needed to survive
         */
        virtual void stop() = 0;

        /*!
         * Get the stop thread variable
         * @return shared_ptr for each each thread;
         */
        const boost::shared_ptr<bool> &getStopThread() const;

        /*!
         * Set the stop thread variable
         * @param stopThread a shared_ptr for all threads
         */
        void setStopThread(const boost::shared_ptr<bool> &stopThread);

    protected:
        boost::shared_ptr<bool> stopThread_; //!< The global shared pointer for stopping all threads
    };

    /*!
     * The vessel base class
     */
    class Vessel : public iVessel {
    public:
        /*!
         * The constructor for a vessel
         * @return
         */
        Vessel();

        /*!
         * The constructor for a vessel
         * @param controller shared_ptr to the controller
         * @return
         */
        Vessel(iController::ptr controller);

        /*!
         * The deconstructor
         */
        virtual ~Vessel();

        /*!
         * Initialize the vessel
         */
        virtual void initialize() override;

        /*!
         * Run the vessel normal operations
         */
        virtual void run() override;

        /*!
         * Stop the vessel, everything except critical parts, which are needed to survive
         */
        virtual void stop() override;

    protected:
        World::ptr world_; //!< a shared_ptr to the world needed for time and location keeping
        iController::ptr controller_; //!< a shared_ptr to the controller needed for sensors, actuators and coommunication
        iCaptain::ptr captain_; //!< The captain for strategical planning
        iBoatswain::ptr boatswain_; //!< The boatswain, the worker asynchronize operations for actuators, sensors, and communication
        std::vector<iSensor::ptr> sensors_; //<! sensor vector
        std::vector<iActuator::ptr> actuators_; //<! actuator vector
        std::vector<iComm::ptr> comm_;
    };

}
