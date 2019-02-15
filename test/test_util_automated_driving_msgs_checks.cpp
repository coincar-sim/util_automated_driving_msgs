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

using namespace util_automated_driving_msgs::checks;
using covariance_type = geometry_msgs::PoseWithCovariance::_covariance_type;

TEST(UtilAutomatedDrivingMsgsChecks, motionStateValidationTest) {

    covariance_type covNonSetValues_{-1, 0, 0, 0,  0, 0, 0, -1, 0, 0, 0,  0, 0, 0, -1, 0, 0, 0,
                                     0,  0, 0, -1, 0, 0, 0, 0,  0, 0, -1, 0, 0, 0, 0,  0, 0, -1};

    covariance_type covGroundTruthValues_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    covariance_type covArbitraryDiagValuesValid_{1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0,
                                                 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 6};

    covariance_type covArbitraryValuesValid_{11, 12, 13, 14, 15, 16, 12, 22, 23, 24, 25, 26, 13, 23, 33, 34, 35, 36,
                                             14, 24, 34, 44, 45, 46, 15, 25, 35, 45, 55, 56, 16, 26, 36, 46, 56, 66};

    covariance_type covInvalidUnsymmetric_{1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18,
                                           19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36};

    covariance_type covInvalidBelowZero_{-11, 12, 13, 14, 15, 16, 12, 22, 23, 24, 25, 26, 13, 23, 33, 34, 35, 36,
                                         14,  24, 34, 44, 45, 46, 15, 25, 35, 45, 55, 56, 16, 26, 36, 46, 56, -66};

    covariance_type covGroundTruthValuesExceptX1_{-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    covariance_type covGroundTruthValuesExceptX2_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0,
                                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0,
                                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0};

    automated_driving_msgs::MotionState ms;

    ms.pose.covariance = covNonSetValues_;
    ms.twist.covariance = covNonSetValues_;
    ms.accel.covariance = covNonSetValues_;
    EXPECT_FALSE(poseValid(ms));
    EXPECT_FALSE(twistValid(ms));
    EXPECT_FALSE(accelValid(ms));
    EXPECT_FALSE(positionValid(ms));
    EXPECT_FALSE(orientationValid(ms));
    EXPECT_FALSE(linearTwistValid(ms));
    EXPECT_FALSE(angularTwistValid(ms));
    EXPECT_FALSE(linearAccelValid(ms));
    EXPECT_FALSE(angularAccelValid(ms));

    ms.pose.covariance = covGroundTruthValues_;
    ms.twist.covariance = covGroundTruthValues_;
    ms.accel.covariance = covGroundTruthValues_;
    EXPECT_TRUE(poseValid(ms));
    EXPECT_TRUE(twistValid(ms));
    EXPECT_TRUE(accelValid(ms));
    EXPECT_TRUE(positionValid(ms));
    EXPECT_TRUE(orientationValid(ms));
    EXPECT_TRUE(linearTwistValid(ms));
    EXPECT_TRUE(angularTwistValid(ms));
    EXPECT_TRUE(linearAccelValid(ms));
    EXPECT_TRUE(angularAccelValid(ms));

    ms.pose.covariance = covArbitraryDiagValuesValid_;
    ms.twist.covariance = covArbitraryDiagValuesValid_;
    ms.accel.covariance = covArbitraryDiagValuesValid_;
    EXPECT_TRUE(poseValid(ms));
    EXPECT_TRUE(twistValid(ms));
    EXPECT_TRUE(accelValid(ms));
    EXPECT_TRUE(positionValid(ms));
    EXPECT_TRUE(orientationValid(ms));
    EXPECT_TRUE(linearTwistValid(ms));
    EXPECT_TRUE(angularTwistValid(ms));
    EXPECT_TRUE(linearAccelValid(ms));
    EXPECT_TRUE(angularAccelValid(ms));

    ms.pose.covariance = covArbitraryValuesValid_;
    ms.twist.covariance = covArbitraryValuesValid_;
    ms.accel.covariance = covArbitraryValuesValid_;
    EXPECT_TRUE(poseValid(ms));
    EXPECT_TRUE(twistValid(ms));
    EXPECT_TRUE(accelValid(ms));
    EXPECT_TRUE(positionValid(ms));
    EXPECT_TRUE(orientationValid(ms));
    EXPECT_TRUE(linearTwistValid(ms));
    EXPECT_TRUE(angularTwistValid(ms));
    EXPECT_TRUE(linearAccelValid(ms));
    EXPECT_TRUE(angularAccelValid(ms));

    ms.pose.covariance = covInvalidUnsymmetric_;
    ms.twist.covariance = covInvalidUnsymmetric_;
    ms.accel.covariance = covInvalidUnsymmetric_;
    EXPECT_FALSE(poseValid(ms));
    EXPECT_FALSE(twistValid(ms));
    EXPECT_FALSE(accelValid(ms));
    EXPECT_FALSE(positionValid(ms));
    EXPECT_FALSE(orientationValid(ms));
    EXPECT_FALSE(linearTwistValid(ms));
    EXPECT_FALSE(angularTwistValid(ms));
    EXPECT_FALSE(linearAccelValid(ms));
    EXPECT_FALSE(angularAccelValid(ms));

    ms.pose.covariance = covInvalidBelowZero_;
    ms.twist.covariance = covInvalidBelowZero_;
    ms.accel.covariance = covInvalidBelowZero_;
    EXPECT_FALSE(poseValid(ms));
    EXPECT_FALSE(twistValid(ms));
    EXPECT_FALSE(accelValid(ms));
    EXPECT_FALSE(positionValid(ms));
    EXPECT_FALSE(orientationValid(ms));
    EXPECT_FALSE(linearTwistValid(ms));
    EXPECT_FALSE(angularTwistValid(ms));
    EXPECT_FALSE(linearAccelValid(ms));
    EXPECT_FALSE(angularAccelValid(ms));

    ms.pose.covariance = covGroundTruthValuesExceptX1_;
    ms.twist.covariance = covGroundTruthValuesExceptX1_;
    ms.accel.covariance = covGroundTruthValuesExceptX1_;
    EXPECT_FALSE(poseValid(ms));
    EXPECT_FALSE(twistValid(ms));
    EXPECT_FALSE(accelValid(ms));
    EXPECT_FALSE(positionValid(ms));
    EXPECT_TRUE(orientationValid(ms));
    EXPECT_FALSE(linearTwistValid(ms));
    EXPECT_TRUE(angularTwistValid(ms));
    EXPECT_FALSE(linearAccelValid(ms));
    EXPECT_TRUE(angularAccelValid(ms));

    ms.pose.covariance = covGroundTruthValuesExceptX2_;
    ms.twist.covariance = covGroundTruthValuesExceptX2_;
    ms.accel.covariance = covGroundTruthValuesExceptX2_;
    EXPECT_FALSE(poseValid(ms));
    EXPECT_FALSE(twistValid(ms));
    EXPECT_FALSE(accelValid(ms));
    EXPECT_TRUE(positionValid(ms));
    EXPECT_FALSE(orientationValid(ms));
    EXPECT_TRUE(linearTwistValid(ms));
    EXPECT_FALSE(angularTwistValid(ms));
    EXPECT_TRUE(linearAccelValid(ms));
    EXPECT_FALSE(angularAccelValid(ms));
}


TEST(UtilAutomatedDrivingMsgsChecks, objectStateArrayValidationTest) {
    automated_driving_msgs::ObjectStateArray osa;
    automated_driving_msgs::ObjectState os;
    automated_driving_msgs::Trajectory traj;
    std::string missingInformationFrameIdsSet{};
    std::string missingInformationFrameIdsValid{};
    std::string missingInformationStampsValid{};
    std::string id_valid = "map";
    std::string id_false = "invalid";
    ros::Time stamp_valid1{503};
    ros::Time stamp_valid2{547};
    ros::Time stamp_invalid{821};
    // objectStateArray
    osa.header.frame_id = id_valid;
    osa.header.stamp = stamp_valid1;
    // objectState
    os.object_id = 0;
    os.header.frame_id = id_valid;
    os.header.stamp = stamp_valid1;
    os.motion_state.header.frame_id = id_valid;
    os.motion_state.header.stamp = stamp_valid1;
    os.motion_state.child_frame_id = id_valid;
    os.motion_prediction.header.frame_id = id_valid;
    os.motion_prediction.header.stamp = stamp_valid1;
    // trajectories
    traj.id = 1;
    automated_driving_msgs::MotionState trajMs1;
    trajMs1.header.frame_id = id_valid;
    trajMs1.child_frame_id = id_valid;
    trajMs1.header.stamp = stamp_valid1;
    traj.motion_states.push_back(trajMs1);
    automated_driving_msgs::MotionState trajMs2;
    trajMs2.header.frame_id = id_valid;
    trajMs2.child_frame_id = id_valid;
    trajMs2.header.stamp = stamp_valid2;
    traj.motion_states.push_back(trajMs2);
    os.motion_prediction.trajectories.push_back(traj);
    osa.objects.push_back(os);

    EXPECT_TRUE(frameIdsSet(osa, missingInformationFrameIdsSet));
    EXPECT_EQ("", missingInformationFrameIdsSet);
    EXPECT_TRUE(frameIdsValid(osa, missingInformationFrameIdsValid));
    EXPECT_EQ("", missingInformationFrameIdsValid);
    EXPECT_TRUE(stampsWithinTimeRange(osa, 60., missingInformationStampsValid));
    EXPECT_EQ("", missingInformationStampsValid);

    osa.objects.begin()->motion_prediction.trajectories.begin()->motion_states.begin()->header.frame_id = id_false;

    EXPECT_TRUE(frameIdsSet(osa, missingInformationFrameIdsSet));
    EXPECT_EQ("", missingInformationFrameIdsSet);
    EXPECT_FALSE(frameIdsValid(osa, missingInformationFrameIdsValid));
    // clang-format off
    EXPECT_EQ("objectStateArray.object[id=0]: motion_prediction.trajectories[id=1]: objectState.motion_prediction.trajectories motionState.header.frame_id is not valid (map != invalid); ",
              missingInformationFrameIdsValid);
    // clang-format on
    EXPECT_TRUE(stampsWithinTimeRange(osa, 60., missingInformationStampsValid));
    EXPECT_EQ("", missingInformationStampsValid);

    osa.objects.begin()->motion_prediction.trajectories.begin()->motion_states.begin()->header.frame_id = id_valid;
    osa.objects.begin()->motion_prediction.trajectories.begin()->motion_states.at(1).header.stamp = stamp_invalid;

    EXPECT_TRUE(frameIdsSet(osa, missingInformationFrameIdsSet));
    EXPECT_EQ("", missingInformationFrameIdsSet);
    EXPECT_TRUE(frameIdsValid(osa, missingInformationFrameIdsValid));
    EXPECT_EQ("", missingInformationFrameIdsValid);
    EXPECT_FALSE(stampsWithinTimeRange(osa, 60., missingInformationStampsValid));
    EXPECT_EQ("Earliest time stamp: 503.000000, latest time stamp: 821.000000", missingInformationStampsValid);

    osa.objects.begin()->motion_prediction.trajectories.begin()->motion_states.at(1).header.stamp = stamp_valid2;
    osa.objects.begin()->motion_prediction.trajectories.begin()->motion_states.at(1).header.frame_id.clear();

    EXPECT_FALSE(frameIdsSet(osa, missingInformationFrameIdsSet));
    // clang-format off
    EXPECT_EQ("objectStateArray.object[id=0]: motion_prediction.trajectories[id=1]: objectState.motion_prediction.trajectories motionState.header.frame_id is not set; ",
              missingInformationFrameIdsSet);
    // clang-format on
    EXPECT_TRUE(stampsWithinTimeRange(osa, 60., missingInformationStampsValid));
    EXPECT_EQ("", missingInformationStampsValid);

    osa.objects.begin()->motion_prediction.trajectories.begin()->motion_states.at(1).header.frame_id = id_valid;
    osa.objects.begin()->motion_prediction.trajectories.begin()->motion_states.at(1).header.stamp = ros::Time{0};

    EXPECT_TRUE(frameIdsSet(osa, missingInformationFrameIdsSet));
    EXPECT_EQ("", missingInformationFrameIdsSet);
    EXPECT_TRUE(frameIdsValid(osa, missingInformationFrameIdsValid));
    EXPECT_EQ("", missingInformationFrameIdsValid);
    EXPECT_FALSE(stampsWithinTimeRange(osa, 60., missingInformationStampsValid));
    EXPECT_EQ("Earliest time stamp is: 0.000000. This should not happen, probably missing stamp initialization.",
              missingInformationStampsValid);
}

TEST(UtilAutomatedDrivingMsgsChecks, predictionStampValidationTest) {

    ros::Time startTime{1000};
    automated_driving_msgs::ObjectStateArray osa;
    osa.header.stamp = startTime;
    std::string missingInformation{};
    const int32_t predictionTimeStepMilliseconds{200};
    const double predictionHorizonSecs{3.1};
    const size_t predictionSteps{16};
    const int64_t predictionTimeStepNanoseconds = predictionTimeStepMilliseconds * 1000000;
    for (size_t i{0}; i < 2; i++) {
        automated_driving_msgs::ObjectState os;
        os.header.stamp = startTime;
        os.object_id = i;
        os.motion_state.header.stamp = startTime;
        os.motion_prediction.header.stamp = startTime;
        for (size_t j{0}; j < 2; j++) {
            automated_driving_msgs::Trajectory traj;
            traj.id = j;
            for (size_t k{0}; k < predictionSteps; k++) {
                automated_driving_msgs::MotionState mS;
                mS.header.stamp = startTime + ros::Duration().fromNSec(int(k) * predictionTimeStepNanoseconds);
                traj.motion_states.push_back(mS);
            }
            os.motion_prediction.trajectories.push_back(traj);
        }
        osa.objects.push_back(os);
    }

    EXPECT_TRUE(stampsWithinTimeRange(osa, 60., missingInformation)) << missingInformation;
    missingInformation.clear();
    EXPECT_TRUE(
        predictionStampsSynchronized(osa, predictionTimeStepMilliseconds, predictionHorizonSecs, missingInformation));
    EXPECT_EQ("", missingInformation);

    osa.objects.front().motion_prediction.trajectories.front().motion_states.at(10).header.stamp += ros::Duration(1);

    EXPECT_FALSE(
        predictionStampsSynchronized(osa, predictionTimeStepMilliseconds, predictionHorizonSecs, missingInformation));
    EXPECT_EQ("Object[id=0] Trajectory[id=0] MotionState[#=10]=1003s0ns deviates from policy "
              "start+n*predictionTimeStep! Expected: 1002s0ns.Moving to next trajectory;  (start=1000s0ns, "
              "step=200000000ns)",
              missingInformation);


    osa.objects.front().motion_prediction.trajectories.front().motion_states.at(10).header.stamp -= ros::Duration(1);
    osa.objects.back().motion_prediction.trajectories.front().motion_states.at(4).header.stamp += ros::Duration(0.5);

    EXPECT_FALSE(
        predictionStampsSynchronized(osa, predictionTimeStepMilliseconds, predictionHorizonSecs, missingInformation));
    EXPECT_EQ("Object[id=1] Trajectory[id=0] MotionState[#=4]=1001s300000000ns deviates from policy "
              "start+n*predictionTimeStep! Expected: 1000s800000000ns.Moving to next trajectory;  (start=1000s0ns, "
              "step=200000000ns)",
              missingInformation);

    osa.objects.back().motion_prediction.trajectories.front().motion_states.at(4).header.stamp -= ros::Duration(0.5);
    osa.objects.back().motion_prediction.trajectories.back().motion_states.at(13).header.stamp += ros::Duration(0.5);

    EXPECT_FALSE(
        predictionStampsSynchronized(osa, predictionTimeStepMilliseconds, predictionHorizonSecs, missingInformation));
    EXPECT_EQ("Object[id=1] Trajectory[id=1] MotionState[#=13]=1003s100000000ns deviates from policy "
              "start+n*predictionTimeStep! Expected: 1002s600000000ns.Moving to next trajectory;  (start=1000s0ns, "
              "step=200000000ns)",
              missingInformation);

    osa.objects.back().motion_prediction.trajectories.back().motion_states.at(13).header.stamp -= ros::Duration(0.5);

    for (auto& object : osa.objects) {
        object.motion_prediction.trajectories.clear();
    }

    EXPECT_TRUE(
        predictionStampsSynchronized(osa, predictionTimeStepMilliseconds, predictionHorizonSecs, missingInformation));
    EXPECT_EQ("", missingInformation);

    osa.objects.clear();

    EXPECT_TRUE(
        predictionStampsSynchronized(osa, predictionTimeStepMilliseconds, predictionHorizonSecs, missingInformation));
    EXPECT_EQ("", missingInformation);
}
