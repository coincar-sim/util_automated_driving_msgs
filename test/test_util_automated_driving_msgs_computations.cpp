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

static const double DoubleTolerance = 10.e-9;

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

    EXPECT_NEAR(1., ms2.twist.twist.linear.x, DoubleTolerance);
    EXPECT_NEAR(0., ms2.twist.twist.linear.y, DoubleTolerance);
    EXPECT_NEAR(0., ms2.twist.twist.linear.z, DoubleTolerance);

    EXPECT_NEAR(0., ms2.accel.accel.linear.x, DoubleTolerance);
    EXPECT_NEAR(0., ms2.accel.accel.linear.y, DoubleTolerance);
    EXPECT_NEAR(0., ms2.accel.accel.linear.z, DoubleTolerance);
}

TEST(UtilAutomatedDrivingMsgsComputations, predictionStampSyncTest) {

    automated_driving_msgs::Trajectory traj{};
    automated_driving_msgs::MotionState ms1;
    automated_driving_msgs::MotionState ms2;

    ms1.header.stamp = ros::Time(1000, 0);
    ms1.pose.pose.position.x = 2.;
    ms1.pose.pose.position.y = 2.;
    ms1.pose.pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 4.);

    ms2.header.stamp = ros::Time(1010, 0);
    ms2.pose.pose.position.x = 3.;
    ms2.pose.pose.position.y = 4.;
    ms2.pose.pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 4.);

    ros::Time interpolationTimeStamp = ros::Time(1005, 0);
    automated_driving_msgs::MotionState interpolatedMotionState;
    size_t actualMotionStateCounter{0};
    bool valid = false;
    std::string errorMsg{};

    size_t indexOfPreviousInterpolation{0};

    // Check no motion states
    interpolateAlongTrajectory(traj,
                               interpolationTimeStamp,
                               actualMotionStateCounter,
                               interpolatedMotionState,
                               indexOfPreviousInterpolation,
                               valid,
                               errorMsg);

    EXPECT_EQ("Not enough motion states available (need at least 2)", errorMsg);
    EXPECT_FALSE(valid);

    // Check interpolation
    errorMsg.clear();
    traj.motion_states.push_back(ms1);
    traj.motion_states.push_back(ms2);
    interpolateAlongTrajectory(traj,
                               interpolationTimeStamp,
                               actualMotionStateCounter,
                               interpolatedMotionState,
                               indexOfPreviousInterpolation,
                               valid,
                               errorMsg);

    EXPECT_EQ("", errorMsg);
    EXPECT_TRUE(valid);
    EXPECT_EQ(2.5, interpolatedMotionState.pose.pose.position.x);
    EXPECT_EQ(3, interpolatedMotionState.pose.pose.position.y);
}

TEST(UtilAutomatedDrivingMsgsComputations, synchronizeObjectPredictionTest) {
    ros::Time startTime{1000};
    const int32_t predictionTimeStepMilliseconds{200};
    const int64_t predictionTimeStepNanoseconds = predictionTimeStepMilliseconds * 1000000;
    automated_driving_msgs::ObjectState os;
    os.header.stamp = startTime;
    os.object_id = 1;
    os.motion_state.header.stamp = startTime;
    os.motion_prediction.header.stamp = startTime;
    for (size_t j{0}; j < 2; j++) {
        automated_driving_msgs::Trajectory traj;
        traj.id = j;
        for (size_t k{0}; k < 15; k++) {
            automated_driving_msgs::MotionState mS;
            mS.header.stamp = startTime + ros::Duration().fromNSec(int(k) * predictionTimeStepNanoseconds);
            mS.pose.pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 4.);
            traj.motion_states.push_back(mS);
        }
        os.motion_prediction.trajectories.push_back(traj);
    }
    automated_driving_msgs::ObjectStateArray osaUnsynced;
    osaUnsynced.header.stamp = startTime;
    osaUnsynced.objects.push_back(os);

    const int newPredictionTimeStep = 500;
    double predictionHorizonSecs = 2.;
    automated_driving_msgs::ObjectStateArray osaSynced;
    bool valid{false};
    std::string errorMsg{};


    synchronizePredictionTimestamps(
        osaUnsynced, newPredictionTimeStep, predictionHorizonSecs, osaSynced, valid, errorMsg);
    EXPECT_TRUE(valid) << errorMsg;

    std::string missingInformation{};
    EXPECT_TRUE(util_automated_driving_msgs::checks::predictionStampsSynchronized(
        osaSynced, newPredictionTimeStep, predictionHorizonSecs, missingInformation))
        << missingInformation;

    predictionHorizonSecs = 10.;
    valid = true;
    errorMsg.clear();
    synchronizePredictionTimestamps(
        osaUnsynced, newPredictionTimeStep, predictionHorizonSecs, osaSynced, valid, errorMsg);
    EXPECT_FALSE(valid) << "Should be out of range";
    EXPECT_EQ(
        "interpolationTimestamp out of range: bigger than traj.motion_states.back().header.stamp; "
        "interpolationTimestamp=1003.000000, trajEndTime=1002.800000 (in step 6 of 21) (in traj_id=1 of obj_id=1)",
        errorMsg);
}
