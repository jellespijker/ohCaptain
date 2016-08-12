//
// Created by peer23peer on 7/15/16.
//

#pragma once

#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/chrono.hpp>
#include <boost/geometry.hpp>

namespace oCpt {
    class World {
    public:
        typedef boost::shared_ptr<World> ptr;

        class Time {
        public:
            typedef boost::shared_ptr<Time> ptr;
            typedef boost::chrono::steady_clock::period tick_period;
            typedef boost::chrono::steady_clock clock_t;
            typedef boost::chrono::time_point<clock_t> timepoint_t;
        private:
            clock_t timeClock_;
        public:
            Time();

            virtual ~Time();

            template<typename T>
            class Log {
            public:
                typedef boost::shared_ptr<Log> ptr;
            private:
                timepoint_t _epoch;
                T _value;
            public:

                Log() {}

                Log(const T &value, const timepoint_t &epoch = clock_t::now()) {
                    this->_value = value;
                    this->_epoch = epoch;
                }

                virtual ~Log() {}

                const timepoint_t &getEpoch() const {
                    return _epoch;
                }

                const T &getValue() const {
                    return _value;
                }
            };

            template<typename T>
            using History = std::vector<boost::shared_ptr<Log<T>>>;

            clock_t &getTimeClock();

            timepoint_t now();
        };

        class Location {
        public:
            typedef boost::shared_ptr<Location> ptr;
            typedef boost::geometry::model::point<
                    double, 2, boost::geometry::cs::spherical_equatorial<boost::geometry::degree>
                    > Earth_point;

            struct RoutePoint {
                typedef boost::shared_ptr<RoutePoint> ptr;
                Time::timepoint_t TimePoint;
                Earth_point Location;
            };

            Location();

            virtual ~Location();

            RoutePoint::ptr getCurrentLocation(bool newMeasurement = false);

            void push_back(RoutePoint::ptr routePoint);

            std::vector<RoutePoint::ptr> getLocationHistory();
        private:
            RoutePoint::ptr currentLocation_;
            std::vector<RoutePoint::ptr> LocationHistory;
        };


        World();

        virtual ~World();

        Time &getTime();

        Time::timepoint_t now();

    protected:
        Time time_;

    };


}
