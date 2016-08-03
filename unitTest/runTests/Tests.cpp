//
// Created by peer23peer on 7/18/16.
//

#include "gtest/gtest.h"

//#include "../../ohCaptain/include/Sensors/PT100.h"
#include "../../ohCaptain/include/Controllers/BeagleboneBlack.h"
#include "../../ohCaptain/include/Vessels/Meetcatamaran.h"

TEST(Meetcatamaran, ADC_read) {
    //oCpt::vessels::Meetcatamaran::ptr mc(new oCpt::vessels::Meetcatamaran());
    //mc->initialize();
    //mc->run();

    EXPECT_EQ(1,1);
}

TEST(Meetcatamaran, SerielTest) {
    oCpt::protocol::Serial serial("/dev/ttyACM0",115200);
    serial.open();
    serial.sig.connect([&]{
        std::cout << serial.readFiFoMsg() << std::endl;
    });
    serial.start();
    for(;;) {}
    //oCpt::World::ptr world_(new oCpt::World());
    //oCpt::components::controller::BBB::ptr bbb(new oCpt::components::controller::BBB(world_));


}