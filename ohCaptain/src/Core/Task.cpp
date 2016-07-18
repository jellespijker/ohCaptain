//
// Created by peer23peer on 7/9/16.
//

#include "../../include/Core/Task.h"

namespace oCpt {
    iTask::iTask(iVessel::ptr vessel, bool concurrent) {
        _concurrent = concurrent;
        _vessel = vessel;
    }

    iTask::~iTask() {}

    iTask::Status::Status() {}

    iTask::Status::~Status() {}

    double iTask::Status::progress() { return _progress; }

    bool iTask::Status::running() { return _running; }

    bool iTask::Status::successful() { return _successful; }

    Task::Task(Vessel::ptr vessel, bool concurrent) : iTask(vessel, concurrent) {
        _status = iTask::Status::ptr(new iTask::Status());
    }

    Task::~Task() {}

    void Task::start() {
        std::for_each(Work.begin(), Work.end(),[&](iTask::ptr &T){

        });
    }

    iTask::Status::ptr Task::status() { return _status; }

    void Task::stop() {}

    RouteTask::RouteTask(Vessel::ptr vessel, bool concurrent) : Task(vessel, concurrent) {
        _typeof = iTask::TypeOf::ROUTE;
    }

    RouteTask::~RouteTask() {}

    WorkTask::WorkTask(Vessel::ptr vessel, bool concurrent) : Task(vessel, concurrent) {
        _typeof = iTask::TypeOf::WORK;
    }

    WorkTask::~WorkTask() {}

    CoveragePathTask::CoveragePathTask(Vessel::ptr vessel, bool concurrent) : RouteTask(vessel, concurrent) {}

    CoveragePathTask::~CoveragePathTask() {}

    FollowTask::FollowTask(Vessel::ptr vessel, bool concurrent) : RouteTask(vessel, concurrent) {}

    FollowTask::~FollowTask() {}

    PathTask::PathTask(Vessel::ptr vessel, bool concurrent) : RouteTask(vessel, concurrent) {}

    PathTask::~PathTask() {}

    LogTask::LogTask(Vessel::ptr vessel, bool concurrent) : WorkTask(vessel, concurrent) {}

    LogTask::~LogTask() {}

    DredgeTask::DredgeTask(Vessel::ptr vessel, bool concurrent) : WorkTask(vessel, concurrent) {}

    DredgeTask::~DredgeTask() {}

    SensorTask::SensorTask(Vessel::ptr vessel, bool concurrent) : WorkTask(vessel, concurrent) {}

    SensorTask::~SensorTask() {}

    ActuatorTask::ActuatorTask(Vessel::ptr vessel, bool concurrent) : WorkTask(vessel, concurrent) {}

    ActuatorTask::~ActuatorTask() {}

    CommunicationTask::CommunicationTask(Vessel::ptr vessel, bool concurrent) : WorkTask(vessel, concurrent) {}

    CommunicationTask::~CommunicationTask() {}
}