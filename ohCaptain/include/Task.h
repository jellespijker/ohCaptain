//
// Created by peer23peer on 7/9/16.
//

#pragma once

#include <boost/shared_ptr.hpp>

namespace oCpt {

    /*! \brief Task interface, all tasks need to adhere to this structure
     *
     * This interface make sure that all task adheres to the same runtime rules and
     * enable run-time polymorphism
     */
    class iTask {
    public:
        typedef boost::shared_ptr<iTask> ptr; //!< Boost shared_ptr to a task

        class Status {
        public:
            typedef boost::shared_ptr<iTask::Status>
                    ptr; //!< Boost shared_ptr to the task status

            /*!
             * Constructor of the iTask
             * @return
             */
            Status();

            /*!
             * Deconstructor
             */
            virtual ~Status();

            /*!
             * Show the progress of the task
             * @return double between 0..1
             */
            double progress();

            /*!
             * Returns the running state of the task
             * @return bool where running is true
             */
            bool running();

            /*!
             * Returns if the task was completed succesfully
             * @return bool where a succesfully completed task is true, task in progress
             * or failed are false
             */
            bool successful();

        private:
            double _progress = 0.0; //<! The current progress of task
            bool _running = false;  //<! Returns true if a task is running
            bool _successful =
                    false; //<! Returns true if a task is completed succesfuly
        };

        /*!
         * Enumeration indicating which type of task the object is
         */
        enum TypeOf {
            Route = 1, Work = 2
        };

        /*!
         * Constructor of the interface
         * @return
         */
        iTask();

        /*!
         * Deconstructor of the interface
         */
        virtual ~iTask();

        /*!
         * The start command for a task
         */
        virtual void start() = 0;

        /*!
         * Retrieves the Status of a task
         * @return Boost shared_ptr of the task status
         */
        virtual iTask::Status::ptr status() = 0;

        /*!
         * The stop command for a task
         */
        virtual void stop() = 0;
    };

    /*!
     * The Base Task class
     */
    class Task : public iTask {
    public:
        /*!
         * The contructor
         * @return
         */
        Task();

        /*!
         * The deconstructor
         */
        virtual ~Task();

        /*!
         * The start command for a task
         */
        virtual void start();

        /*!
         * Retrieves the Status of a task
         * @return Boost shared_ptr of the task status
         */
        virtual iTask::Status::ptr status();

        /*!
         * The stop command for a task
         */
        virtual void stop();

    protected:
        iTask::Status::ptr _status; //!< a boost share_ptr to the status of a task
        TypeOf _typeof;             //!< Indicating the type of a task
    };

    /*!
     * An object repsressenting route related tasks
     */
    class RouteTask : public Task {
    public:
        /*!
         * Constructor of the interface
         * @return
         */
        RouteTask();
        
        /*!
         * The deconstructor
         */
        virtual ~RouteTask();

    protected:
    };

    /*!
     * An object representing work related tasks
     */
    class WorkTask : public Task {
    public:
        /*!
         * Constructor of the interface
         * @return
         */
        WorkTask();

        /*!
         * The deconstructor
         */
        virtual ~WorkTask();

    protected:
    };

    /*!
     * \brief An object representing a coverage path task
     *
     *  All these types of tasks need a robot to cover a complete region in order to
     * perform their tasks. According to \citet{cao_region_1988} such a mobile robot
     * should use the following criteria, for a region filling operation:
     *  1. The mobile robot must move through an entire area, i.e., the overall
     * travel must cover a whole region.
     *  2. The mobile robot must fill the region without overlapping paths.
     *  3. Continuous and sequential operations without any repetition of paths is
     * required of the robot.
     *  4. The robot must avoid all obstacles in a region.
     *  5. Simple motion trajectories (e.g., straight lines or circles) should be
     * used for simplicity in control.
     *  6. An "optimal" path is desired under the available conditions. It is not
     * always possible to satisfy all these criteria for a complex environment.
     * Sometimes a priority consideration is required.
     */
    class CoveragePathTask : public RouteTask {
    public:
        /*!
        * Constructor of the interface
        * @return
        */
        CoveragePathTask();

        /*!
         * The deconstructor
         */
        virtual ~CoveragePathTask();

    protected:
    };

    /*!
     * \brief An object representing a follow the target task
     *
     * All these types of tasks need to follow a (moving) target
     */
    class FollowTask : public RouteTask {
    public:
        /*!
         * Constructor of the interface
         * @return
         */
        FollowTask();

        /*!
         * The deconstructor
         */
        virtual ~FollowTask();

    protected:
    };

    /*!
     * \brief An object representing a normal A to B type of path planning
     *
     * All these types of tasks need to plann an optimum route between A and B,
     * either in time, energy consumption or
     */
    class PathTask : public RouteTask {
    public:
        /*!
         * Constructor of the interface
         * @return
         */
        PathTask();

        /*!
         * The deconstructor
         */
        virtual ~PathTask();

    protected:
    };

    /*!
     * \brief An Object representing a data logging task
     *
     * All these types of tasks make use of a sensor to record and log
     */
    class LogTask : public WorkTask {
    public:
        /*!
        * Constructor of the interface
        * @return
        */
        LogTask();

        /*!
         * The deconstructor
         */
        virtual ~LogTask();

    protected:
    };

    /*!
     * \brief An Object representing a dredging task
     *
     * All these types tasks make use of an actuator and sensors to perform dredging
     * tasks
     */
    class DredgeTask : public WorkTask {
    public:
        /*!
         * Constructor of the interface
         * @return
         */
        DredgeTask();

        /*!
         * The deconstructor
         */
        virtual ~DredgeTask();

    protected:
    };
}
