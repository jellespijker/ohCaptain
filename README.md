# ohCaptain

*MTI project number: * **21517**

A generic library written in C++ for embedded devices (such as the Raspberry Pi or Beaglebone) which implements **Artificial Intelligence for (under)water vessels**. Such that they can act and plan their own route. Taking in to account their environment through different sensors and adjusting their course with different actuators.

The goal is to write it as generic a possible, such that it can be used by floating, gliding, riding or walking. This is done by modeling the actuators and sensors via an interface and passing them a pointers to the captain object.

## Project structure
* The project has an actively maintained [Wiki](http://mti.isa-geek.com/peer23peer/captain/wikis/home) for literature and explanation of concepts.
* Bugs, features and enhancement are logged as [Issues](http://mti.isa-geek.com/peer23peer/captain/milestones)
* Planning is done with [Milestones](http://mti.isa-geek.com/peer23peer/captain/milestones)

## Work flow
GitFlow specifies a standard branch structure and workflow for moving changes between branches, which standardise the way a team uses Git, and keep the repository clean and structured, with important branches protected from ad hoc changes.

The purpose of a branch and its position in the workflow, from development to release, is indicated by its name:

* Release/{branch name} – each release branch corresponds to a released version of the application
* Master – a permanent branch acting as a definitive record of the latest released version of the application
* Develop – a permanent branch containing the work-in-progress version of the site, with feature branches being created from this branch and merged back into it
* Feature/{branch name} – each feature branch corresponds to work on a particular area (e.g. a bugfix or new feature)
* Hotfix/{branch name} – branches for dealing with urgent fixes, which get merged into master (and develop) and then a new release branch when complete.


The workflow between branches is shown in the visualisation below, where circles represent commits, and arrows merges. .

![Gitflow](https://blogs.endjin.com/wp-content/uploads/2015/01/GitFlowworkflow2.png)

## Dependancies
* [Boost](http://www.boost.org/)
* [CMake](https://cmake.org/)

## Used in projects
* [Autonomous Dredge Bot](http://mti.isa-geek.com/AOD/ADB)
* [Pompilos](http://mti.isa-geek.com/peer23peer/pompilos)
