//
// Created by peer23peer on 7/18/16.
//

#include "gtest/gtest.h"

//#include "../../ohCaptain/include/Sensors/PT100.h"
//#include "../../ohCaptain/include/Controllers/BeagleboneBlack.h"
#include "../../ohCaptain/include/Vessels/Meetcatamaran.h"

TEST(Meetcatamaran, ADC_read) {
    oCpt::vessels::Meetcatamaran::ptr mc(new oCpt::vessels::Meetcatamaran());
    mc->run();

    EXPECT_EQ(1,1);
}