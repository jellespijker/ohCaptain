#pragma once

#include "Razor.h"

#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/systems/si/acceleration.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/length.hpp>

#include "../../../thirdparty/kalman/include/kalman/LinearizedSystemModel.hpp"
#include "../../../thirdparty/kalman/include/kalman/ExtendedKalmanFilter.hpp"

namespace oCpt {
    namespace components {
        namespace sensors {
            template<typename T>
            class StateKalmanIMU : Kalman::Vector<T, 15> {
            public:
                KALMAN_VECTOR(StateKalmanIMU, T, 15)

                static constexpr size_t posX = 0;
                static constexpr size_t posY = 1;
                static constexpr size_t posZ = 2;
                static constexpr size_t velX = 3;
                static constexpr size_t velY = 4;
                static constexpr size_t velZ = 5;
                static constexpr size_t accX = 6;
                static constexpr size_t accY = 7;
                static constexpr size_t accZ = 8;
                static constexpr size_t Theta = 9;
                static constexpr size_t Psi = 10;
                static constexpr size_t Phi = 11;
                static constexpr size_t ThetaPrime = 12;
                static constexpr size_t PsiPrime = 13;
                static constexpr size_t PhiPrime = 14;

                T pos_x() const { return (*this)[posX]; }

                T pos_y() const { return (*this)[posY]; }

                T pos_z() const { return (*this)[posZ]; }

                T vel_x() const { return (*this)[velX]; }

                T vel_y() const { return (*this)[velY]; }

                T vel_z() const { return (*this)[velZ]; }

                T acc_x() const { return (*this)[accX]; }

                T acc_y() const { return (*this)[accY]; }

                T acc_z() const { return (*this)[accZ]; }

                T theta() const { return (*this)[Theta]; }

                T psi() const { return (*this)[Psi]; }

                T phi() const { return (*this)[Phi]; }

                T thetaPrime() const { return (*this)[ThetaPrime]; }

                T psiPrime() const { return (*this)[PsiPrime]; }

                T phiPrime() const { return (*this)[PhiPrime]; }

                T &pos_x() { return (*this)[posX]; }

                T &pos_y() { return (*this)[posY]; }

                T &pos_z() { return (*this)[posZ]; }

                T &vel_x() { return (*this)[velX]; }

                T &vel_y() { return (*this)[velY]; }

                T &vel_z() { return (*this)[velZ]; }

                T &acc_x() { return (*this)[accX]; }

                T &acc_y() { return (*this)[accY]; }

                T &acc_z() { return (*this)[accZ]; }

                T &theta() { return (*this)[Theta]; }

                T &psi() { return (*this)[Psi]; }

                T &phi() { return (*this)[Phi]; }

                T &thetaPrime() { return (*this)[ThetaPrime]; }

                T &psiPrime() { return (*this)[PsiPrime]; }

                T &phiPrime() { return (*this)[PhiPrime]; }
            };

            template<typename T>
            class ControlKalmanIMU : public Kalman::Vector<T, 1> {
            public:
                KALMAN_VECTOR(ControlKalmanIMU, T, 1)
                //TODO determine if a Control class is needed
            };

            template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
            class SystemModelKalmanIMU
                    : public Kalman::LinearizedSystemModel<StateKalmanIMU<T>, ControlKalmanIMU<T>, CovarianceBase> {
            public:
                typedef StateKalmanIMU<T> S;
                typedef ControlKalmanIMU<T> C;

                S f(const S &x, const C &u) const {
                    S x_;
                    auto newOrientation = 0; //TODO workout further
                }
            };

            template<typename T>
            class OrientationMeasurementKalmanIMU : public Kalman::Vector<T, 6> {
            public:
                KALMAN_VECTOR(OrientationMeasurementKalmanIMU, T, 6)

                static constexpr size_t Theta = 0;
                static constexpr size_t Psi = 1;
                static constexpr size_t Phi = 2;
                static constexpr size_t ThetaPrime = 3;
                static constexpr size_t PsiPrime = 4;
                static constexpr size_t PhiPrime = 5;

                T theta() const { return (*this)[Theta]; }

                T psi() const { return (*this)[Psi]; }

                T phi() const { return (*this)[Phi]; }

                T thetaPrime() const { return (*this)[ThetaPrime]; }

                T psiPrime() const { return (*this)[PsiPrime]; }

                T phiPrime() const { return (*this)[PhiPrime]; }

                T &theta() { return (*this)[Theta]; }

                T &psi() { return (*this)[Psi]; }

                T &phi() { return (*this)[Phi]; }

                T &thetaPrime() { return (*this)[ThetaPrime]; }

                T &psiPrime() { return (*this)[PsiPrime]; }

                T &phiPrime() { return (*this)[PhiPrime]; }
            };

            template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
            class OrientationMeasurementModelKalmanIMU
                    : public Kalman::LinearizedMeasurementModel<StateKalmanIMU<T>, OrientationMeasurementModelKalmanIMU<T>, CovarianceBase> {
            public:
                typedef StateKalmanIMU<T> S;

                typedef OrientationMeasurementKalmanIMU<T> M;

                OrientationMeasurementModelKalmanIMU() {
                    this->H.setIdentity();
                    this->V.setIdentity();
                }

                M h(const S &x) const {
                    M measurement;

                    //TODO determine if sensor reading should be inputed in here or that I can make an new StateKalmanIMU<T> and input it
                    measurement.theta() = x.theta();
                    measurement.phi() = x.phi();
                    measurement.psi() = x.psi();
                    measurement.thetaPrime() = x.thetaPrime();
                    measurement.phiPrime() = x.phiPrime();
                    measurement.psiPrime() = x.psiPrime();

                    return measurement;
                }
            };

            template<typename T>
            class PositionMeasurementKalmanIMU : public Kalman::Vector<T, 3> {
            public:
                KALMAN_VECTOR(PositionMeasurementKalmanIMU, T, 3)

                static constexpr size_t accX = 0;
                static constexpr size_t accY = 1;
                static constexpr size_t accZ = 2;

                T acc_x() const { return (*this)[accX]; }

                T acc_y() const { return (*this)[accY]; }

                T acc_z() const { return (*this)[accZ]; }

                T &acc_x() { return (*this)[accX]; }

                T &acc_y() { return (*this)[accY]; }

                T &acc_z() { return (*this)[accZ]; }
            };

            template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
            class PositionMeasurementModelKalmanIMU
                    : public Kalman::LinearizedMeasurementModel<StateKalmanIMU<T>, PositionMeasurementKalmanIMU<T>, CovarianceBase> {
            public:
                typedef StateKalmanIMU<T> S;

                typedef  PositionMeasurementKalmanIMU<T> M;

                PositionMeasurementModelKalmanIMU() {
                    this->V.setIdentity();
                }

                M h(const S& x) const {
                    M measurement;

                    measurement.acc_x() = x.acc_x();
                    measurement.acc_y() = x.acc_y();
                    measurement.acc_z() = x.acc_z();

                    return measurement;
                }
            };

            class KalmanIMU : public Sensor {
            public:
                typedef struct ReturnValue {
                    quantity<si::length, double> position[2]; //<! Position in X and Y
                    quantity<si::velocity, double> velocity[2]; //<! Velocity in X and Y direction
                    quantity<si::acceleration, double> acceleration[2]; //<! Acceleration in X and Y direction
                    quantity<si::plane_angle, double> orientation[3]; //<! Orientation, pitch, roll, yaw
                    quantity<angular_velocity_dimension, double> orientation_change[3]; // Rate of change in orientation
                } ReturnValue_t;

                KalmanIMU(iController::ptr controller, World::ptr world, std::string id, Razor::ptr RazorSensor);

                ~KalmanIMU();

                void updateSensor();

                void run();

                void init();

                void setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice);

            private:
                std::string device_;
                Razor::ptr razor_;

                void RazorUpdate();
            };
        }
    }
}
