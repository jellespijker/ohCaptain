//
// Created by peer23peer on 7/18/16.
//

#include "gtest/gtest.h"

//#include "../../ohCaptain/include/Sensors/PT100.h"
#include "../../ohCaptain/include/Controllers/BeagleboneBlack.h"
#include "../../ohCaptain/include/Vessels/Meetcatamaran.h"
#include "../../ohCaptain/include/Communication/LoRa_RN2483.h"

//TEST(Meetcatamaran, SerielTest) {
//    oCpt::protocol::Serial serial("/dev/ttyACM0", 57600);
//    serial.open();
//    serial.msgRecievedSig.connect([&]{
//        std::cout << serial.readFiFoMsg() << std::endl; });
//    serial.start();
//    serial.write("test");
//    for(;;) {}
//}

//TEST(Meetcatamaran, ADC_read) {
//    //oCpt::vessels::Meetcatamaran::ptr mc(new oCpt::vessels::Meetcatamaran());
//    //mc->initialize();
//    //mc->run();
//
//    EXPECT_EQ(1,1);
//}
//
TEST(Meetcatamaran, LoRaTest) {
    oCpt::components::comm::LoRa_RN2483 loRaRn2483("loRaRn2483", "/dev/ttyACM0");
    loRaRn2483.initialize();
    loRaRn2483.msgRecievedSig.connect([&]{
        std::cout << loRaRn2483.readFiFoMsg()->Payload << std::endl;
    });
    loRaRn2483.run();
    for(;;);
    loRaRn2483.stop();
}

