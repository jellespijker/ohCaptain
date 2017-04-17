#include "../../include/Sensors/KalmanIMU.h"

namespace oCpt {
    namespace components {
        namespace sensors {

            KalmanIMU::KalmanIMU(iController::ptr controller, World::ptr world, std::string id,
                                 Razor::ptr RazorSensor) : Sensor(controller, world, id, "IMU"),
                                                           razor_(RazorSensor) {
                razor_->getSig().connect(boost::bind(&KalmanIMU::RazorUpdate, this));

            }

            KalmanIMU::~KalmanIMU() {

            }

            void KalmanIMU::updateSensor() {
                Sensor::updateSensor();
            }

            void KalmanIMU::run() {
                Sensor::run();
            }

            void KalmanIMU::init() {
                Sensor::init();
            }

            void KalmanIMU::setIOservice(boost::shared_ptr<boost::asio::io_service> ioservice) {
                Sensor::setIOservice(ioservice);
            }

            void KalmanIMU::RazorUpdate() {
                Razor::ReturnValue_t razRet = CAST(razor_->getState().Value, Razor);
                std::cout << razRet.acc[0] << ", " << razRet.acc[1] << ", " <<  razRet.acc[2] << std::endl;
            }
        }
    }
}
