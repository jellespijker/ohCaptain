//
// Created by peer23peer on 7/15/16.
//

#pragma once

#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/chrono.hpp>

namespace oCpt {
    class World {
    public:
        typedef boost::shared_ptr<World> ptr;

        class Time {
        private:
            boost::chrono::steady_clock timeClock_;
        public:
            typedef boost::shared_ptr<Time> ptr;
            typedef boost::chrono::steady_clock::period tick_period;

            Time();

            virtual ~Time();

            template<typename T>
            class Log {
            public:
                typedef boost::shared_ptr<Log> ptr;
                typedef boost::chrono::steady_clock::time_point tp;
            private:
                tp _epoch;
                T _value;
            public:

                Log() {}

                Log(const T &value, const tp &epoch = boost::chrono::steady_clock::now()) {
                    this->_value = value;
                    this->_epoch = epoch;
                }

                virtual ~Log() {}

                const tp &getEpoch() const {
                    return _epoch;
                }

                const T &getValue() const {
                    return _value;
                }
            };

            template<typename T>
            using History = std::vector<boost::shared_ptr<Log<T>>>;

            boost::chrono::steady_clock &getTimeClock();

            boost::chrono::steady_clock::time_point now();
        };

        World();

        virtual ~World();

        Time &getTime();

    protected:
        Time time_;

    };


}
