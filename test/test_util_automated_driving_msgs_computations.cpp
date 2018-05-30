/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "gtest/gtest.h"

#include "util_automated_driving_msgs.hpp"

using namespace util_automated_driving_msgs::computations;

static const double DOUBLE_TOLERANCE = 10.e-9;

TEST(UtilAutomatedDrivingMsgsComputations, incorporatePrecedingDataToMotionstate) {
    automated_driving_msgs::MotionState ms0;
    ms0.pose.covariance = util_geometry_msgs::checks::covarianceGroundTruthValues;
    ms0.pose.pose.position.z = 0.;
    ms0.twist.covariance = util_geometry_msgs::checks::covarianceUnkownValues;
    ms0.accel.covariance = util_geometry_msgs::checks::covarianceUnkownValues;

    ms0.pose.pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 4.);

    ms0.header.stamp.fromSec(0.);
    ms0.pose.pose.position.x = 1.;
    ms0.pose.pose.position.y = 1.;

    automated_driving_msgs::MotionState ms1{ms0};
    ms1.header.stamp.fromSec(sqrt(2.));
    ms1.pose.pose.position.x = 2.;
    ms1.pose.pose.position.y = 2.;

    automated_driving_msgs::MotionState ms2{ms0};
    ms2.header.stamp.fromSec(2. * sqrt(2.));
    ms2.pose.pose.position.x = 3.;
    ms2.pose.pose.position.y = 3.;

    incorporatePrecedingDataToMotionstate(ms0, ms1);
    incorporatePrecedingDataToMotionstate(ms1, ms2);

    EXPECT_TRUE(util_automated_driving_msgs::checks::accelValid(ms2));
    EXPECT_TRUE(util_automated_driving_msgs::checks::twistValid(ms2));

    EXPECT_NEAR(1., ms2.twist.twist.linear.x, DOUBLE_TOLERANCE);
    EXPECT_NEAR(0., ms2.twist.twist.linear.y, DOUBLE_TOLERANCE);
    EXPECT_NEAR(0., ms2.twist.twist.linear.z, DOUBLE_TOLERANCE);

    EXPECT_NEAR(0., ms2.accel.accel.linear.x, DOUBLE_TOLERANCE);
    EXPECT_NEAR(0., ms2.accel.accel.linear.y, DOUBLE_TOLERANCE);
    EXPECT_NEAR(0., ms2.accel.accel.linear.z, DOUBLE_TOLERANCE);
}
