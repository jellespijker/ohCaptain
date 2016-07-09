//
// Created by peer23peer on 7/9/16.
//

#include "../include/Task.h"

namespace oCpt {
    iTask::iTask() {}

    iTask::~iTask() {}

    iTask::Status::Status() {}

    iTask::Status::~Status() {}

    double iTask::Status::progress() { return _progress; }

    bool iTask::Status::running() { return _running; }

    bool iTask::Status::successful() { return _successful; }

    Task::Task() { _status = iTask::Status::ptr(new iTask::Status()); }

    Task::~Task() {}

    void Task::start() {}

    iTask::Status::ptr Task::status() { return _status; }

    void Task::stop() {}

    RouteTask::RouteTask() { _typeof = iTask::TypeOf::Route; }

    RouteTask::~RouteTask() {}

    WorkTask::WorkTask() { _typeof = iTask::TypeOf::Work; }

    WorkTask::~WorkTask() {}

    CoveragePathTask::CoveragePathTask() {}

    CoveragePathTask::~CoveragePathTask() {}

    FollowTask::FollowTask() {}

    FollowTask::~FollowTask() {}

    PathTask::PathTask() {}

    PathTask::~PathTask() {}

    LogTask::LogTask() {}

    LogTask::~LogTask() {}

    DredgeTask::DredgeTask() {}

    DredgeTask::~DredgeTask() {}
}