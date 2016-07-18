//
// Created by peer23peer on 7/18/16.
//

#include "gtest/gtest.h"

#include "../../ohCaptain/include/Core/Exception.h"
#include "../../ohCaptain/include/Core/Controller.h"
#include "../../ohCaptain/include/Core/Task.h"

TEST(ADC, adc_compare) {
    oCpt::protocol::adc compADCa(0,0);
    oCpt::protocol::adc compADCb(0,0);
    EXPECT_TRUE(compADCa == compADCb);
}

TEST(ADC, voltage) {
    oCpt::BBB::ptr test( new oCpt::BBB() );
    oCpt::protocol::adc::ptr ADC = test->getAdcVector()->at(0);
    uint16_t  value = ADC->getValue();
    EXPECT_LE(value, 1) << "Failed! is there a voltage applied to the ADC port?";
}

TEST(TASK, periodic_check_adc_read) {
    oCpt::BBB::ptr bbb( new oCpt::BBB() );
    oCpt::Vessel::ptr testVessel( new oCpt::Vessel() );
    oCpt::iTask::ptr pCheck( new oCpt::SensorTask(testVessel, false) );
    pCheck->start();
}