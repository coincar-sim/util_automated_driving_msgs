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

#include "util_automated_driving_msgs.hpp"

namespace util_automated_driving_msgs {

namespace conversions {

automated_driving_msgs::ObjectState objectStateFromObjectStateArray(
    const automated_driving_msgs::ObjectStateArray& inputObjectArray, uint32_t objectId, bool& foundAndUnique) {

    foundAndUnique = false;
    automated_driving_msgs::ObjectState objectState;

    for (size_t i = 0; i < inputObjectArray.objects.size(); i++) {
        if (inputObjectArray.objects[i].object_id == objectId) {

            if (foundAndUnique) {
                // if object already found, its id is not unique
                foundAndUnique = false;
                return objectState;
            } else {
                // if not found yet, it is found now
                foundAndUnique = true;
                objectState = inputObjectArray.objects[i];
            }
        }
    }
    return objectState;
}

automated_driving_msgs::ObjectState objectStateFromObjectStateArray(
    boost::shared_ptr<const automated_driving_msgs::ObjectStateArray> inputObjectArray,
    uint32_t objectId,
    bool& foundAndUnique) {

    return objectStateFromObjectStateArray(*inputObjectArray, objectId, foundAndUnique);
}

automated_driving_msgs::ObjectStateArray removeObjectFromObjectStateArray(
    const automated_driving_msgs::ObjectStateArray& inputObjectArray, uint32_t objectId) {

    automated_driving_msgs::ObjectStateArray outputObjectArray = inputObjectArray;

    for (size_t i = 0; i < inputObjectArray.objects.size(); i++) {
        if (inputObjectArray.objects[i].object_id == objectId) {
            outputObjectArray.objects.erase(outputObjectArray.objects.begin() + i);
            return outputObjectArray;
        }
    }
    return outputObjectArray;
}

} // namespace util_automated_driving_msgs::conversions

namespace checks {

bool positionValid(const automated_driving_msgs::MotionState& motionState) {
    const boost::array<double, 36>& cov = motionState.pose.covariance;
    // Checking first three diagonal elements.
    return ((cov[0] >= 0) && (cov[7] >= 0) && (cov[14] >= 0) &&
            util_geometry_msgs::checks::topLeft3x3CovarianceMatrixValid(cov));
}

bool orientationValid(const automated_driving_msgs::MotionState& motionState) {
    const boost::array<double, 36>& cov = motionState.pose.covariance;
    // Checking last three diagonal elements.
    return ((cov[21] >= 0) && (cov[28] >= 0) && (cov[35] >= 0) &&
            util_geometry_msgs::checks::bottomRight3x3CovarianceMatrixValid(cov));
}

bool poseValid(const automated_driving_msgs::MotionState& motionState) {
    return (positionValid(motionState) && orientationValid(motionState) &&
            util_geometry_msgs::checks::covarianceMatrixValid(motionState.pose.covariance));
}

bool linearTwistValid(const automated_driving_msgs::MotionState& motionState) {
    const boost::array<double, 36>& cov = motionState.twist.covariance;
    // Checking first three diagonal elements.
    return ((cov[0] >= 0) && (cov[7] >= 0) && (cov[14] >= 0) &&
            util_geometry_msgs::checks::topLeft3x3CovarianceMatrixValid(cov));
}

bool angularTwistValid(const automated_driving_msgs::MotionState& motionState) {
    const boost::array<double, 36>& cov = motionState.twist.covariance;
    // Checking last three diagonal elements.
    return ((cov[21] >= 0) && (cov[28] >= 0) && (cov[35] >= 0) &&
            util_geometry_msgs::checks::bottomRight3x3CovarianceMatrixValid(cov));
}

bool twistValid(const automated_driving_msgs::MotionState& motionState) {
    return (linearTwistValid(motionState) && angularTwistValid(motionState) &&
            util_geometry_msgs::checks::covarianceMatrixValid(motionState.twist.covariance));
}

bool linearAccelValid(const automated_driving_msgs::MotionState& motionState) {
    const boost::array<double, 36>& cov = motionState.accel.covariance;
    // Checking first three diagonal elements.
    return ((cov[0] >= 0) && (cov[7] >= 0) && (cov[14] >= 0) &&
            util_geometry_msgs::checks::topLeft3x3CovarianceMatrixValid(cov));
}

bool angularAccelValid(const automated_driving_msgs::MotionState& motionState) {
    const boost::array<double, 36>& cov = motionState.accel.covariance;
    // Checking last three diagonal elements.
    return ((cov[21] >= 0) && (cov[28] >= 0) && (cov[35] >= 0) &&
            util_geometry_msgs::checks::bottomRight3x3CovarianceMatrixValid(cov));
}

bool accelValid(const automated_driving_msgs::MotionState& motionState) {
    return (linearAccelValid(motionState) && angularAccelValid(motionState) &&
            util_geometry_msgs::checks::covarianceMatrixValid(motionState.accel.covariance));
}

} // namespace util_automated_driving_msgs::checks

namespace computations {

/** @brief Incorporate the Data (in this Case the POSE) of a preceding MotionState to a more recent MotionState.
 * NOTE: Helper-method which is only to be used in this very .cpp.
 * @param[in] The first/preceding (older) MotionState. This MotionState is never changed.
 * @param[in,out] The second (present, more recent, newer) MotionState. This MotionState is changed, complemented by the
 * Data in the preceding Motionstate
 * @return true if the second MotionState has been changed, false if not or on failure. */
static bool incorporatePrecedingPoseToTwist(const automated_driving_msgs::MotionState& precedingMS,
                                            automated_driving_msgs::MotionState& presentMS) {

    // Validate Input
    if (precedingMS.header.frame_id != presentMS.header.frame_id) {
        ROS_DEBUG("Could not calculate twist: frame_ids are not equal. Twist remains unchanged.");
        return false;
    }
    if (!(checks::poseValid(precedingMS) && checks::poseValid(presentMS))) {
        ROS_DEBUG("Could not calculate twist: Poses are not valid. Twist remains unchanged.");
        return false;
    }

    // Calculate DeltaTime between the to MotionStates
    const double deltaTime = (presentMS.header.stamp - precedingMS.header.stamp).toSec();
    if (std::abs(deltaTime) < 10.e-9) {
        ROS_DEBUG("Could not calculate twist: DeltaTime is zero. Twist remains unchanged.");
        return false;
    }

    // Get Frame in which the Twist is expressed.
    const geometry_msgs::Pose& presentChildFrame = presentMS.pose.pose;

    // define base_frame
    geometry_msgs::Pose baseFrame;
    baseFrame.position.x = 0.0;
    baseFrame.position.y = 0.0;
    baseFrame.position.z = 0.0;
    baseFrame.orientation.x = 0.0;
    baseFrame.orientation.y = 0.0;
    baseFrame.orientation.z = 0.0;
    baseFrame.orientation.w = 1.0;
    baseFrame.orientation.w = 1.0;

    // Transform preceding Pose from base frame in childFrame defined by presentMS.header.child_frame_id
    geometry_msgs::PoseWithCovariance transformedPose0, transformedPose1;
    util_geometry_msgs::transformations::rereferencePoseWithCovariance(
        precedingMS.pose, baseFrame, presentChildFrame, transformedPose0);

    // Transform present Pose from base frame in childFrame defined by presentMS.header.child_frame_id
    // The childFrame is defined by the pose itself
    // Hence the Pose can be hardcoded to position(0,0,0) and rotation to zero (0,0,0,1) as quaternion,
    // This is (except numerical issues) equivalent to calling
    // util_perception::rereferencePoseWithCovariance(presentMS.pose, presentChildFrame, transformedPose1);
    transformedPose1.pose.position.x = 0.0;
    transformedPose1.pose.position.y = 0.0;
    transformedPose1.pose.position.z = 0.0;
    transformedPose1.pose.orientation.x = 0.0;
    transformedPose1.pose.orientation.y = 0.0;
    transformedPose1.pose.orientation.z = 0.0;
    transformedPose1.pose.orientation.w = 1.0;
    // Only the transformation of the covariance matrix has to be calculated.
    util_geometry_msgs::transformations::rereferenceCovariance(
        presentMS.pose.covariance, baseFrame, presentChildFrame, transformedPose1.covariance);

    // Calculate difference quotient as approximation for twist
    // Keep in mind that the angular velocity is not unambiguously assignable when calculated from two
    // orientations
    geometry_msgs::Twist& resultingTwist = presentMS.twist.twist;
    diffQuotientXYZ(transformedPose0.pose.position, transformedPose1.pose.position, deltaTime, resultingTwist.linear);
    diffQuotientOrientation(
        transformedPose0.pose.orientation, transformedPose1.pose.orientation, deltaTime, resultingTwist.angular);

    // Covariance
    // Assumptions:
    // * both poses are uncorrelated, thus Cov(Pose0,Pose1)=0
    // * The Deltatime is exactly known (in the sense that the uncertainty can be neglected compared to the
    //   uncertainty of the poses, thus the deltaTime can be treated as constant with no uncertainty).
    // This leads to covariance_twist = (1/deltaTime)^2 * (covariance_pose0 + covariance_pose1)
    geometry_msgs::TwistWithCovariance::_covariance_type& resultingCovariance = presentMS.twist.covariance;
    cWiseAdd(transformedPose0.covariance, transformedPose1.covariance, resultingCovariance);
    cWiseDivide(resultingCovariance, (1 / (deltaTime * deltaTime)), resultingCovariance);

    return true;
}

/** @brief Incorporate the Data (in this Case the TWIST) of a preceding MotionState to a more recent MotionState.
 * NOTE: Helper-method which is only to be used in this very .cpp.
 * @param[in] The first/preceding (older) MotionState. This MotionState is never changed.
 * @param[in,out] The second (present, more recent, newer) MotionState. This MotionState is changed, complemented by the
 * Data in the preceding Motionstate
 * @return true if the second MotionState has been changed, false if not or on failure. */
static bool incorporatePrecedingTwistToAccel(const automated_driving_msgs::MotionState& precedingMS,
                                             automated_driving_msgs::MotionState& presentMS) {
    // Validate Input
    if (precedingMS.header.frame_id != presentMS.header.frame_id) {
        ROS_DEBUG("Could not calculate acceleration: frame_ids are not equal. Acceleration remains unchanged.");
        return false;
    }
    if (!(checks::twistValid(precedingMS) && checks::twistValid(presentMS))) {
        ROS_DEBUG("Could not calculate acceleration: Twists are not valid. Acceleration remains unchanged.");
        return false;
    }

    // Calculate DeltaTime between the to MotionStates
    const double deltaTime = (presentMS.header.stamp - precedingMS.header.stamp).toSec();
    if (std::abs(deltaTime) < 10.e-9) {
        ROS_DEBUG("Could not calculate acceleration: DeltaTime is zero. Acceleration remains unchanged.");
        return false;
    }

    // Get Frame in which the Acceleration is expressed.
    const geometry_msgs::Pose& presentChildFrame = presentMS.pose.pose;

    // Transform twists in frame defined by presentMS.header.child_frame_id
    const geometry_msgs::Pose& precedingChildFrame = precedingMS.pose.pose;
    geometry_msgs::TwistWithCovariance transformedTwist0;
    util_geometry_msgs::transformations::rereferenceTwistWithCovariance(
        precedingMS.twist, precedingChildFrame, presentChildFrame, transformedTwist0);

    // twist1 is already in correct frame
    const geometry_msgs::TwistWithCovariance& twist1 = presentMS.twist;

    // differentiate
    geometry_msgs::Accel& resultingAccel = presentMS.accel.accel;
    diffQuotientXYZ(transformedTwist0.twist.linear, twist1.twist.linear, deltaTime, resultingAccel.linear);
    diffQuotientXYZ(transformedTwist0.twist.angular, twist1.twist.angular, deltaTime, resultingAccel.angular);

    // Covariance
    // Assumptions:
    // * both twists are uncorrelated, thus Cov(Twist0,Twist1)=0
    // * The Deltatime is exactly known (in the sense that the uncertainty can be neglected compared to the
    //     uncertainty of the poses, thus the deltaTime can be treated as constant with no uncertainty.
    // This leads to covariance_accel = (1/deltaTime)^2 * (covariance_twist0 + covariance_wist1)
    geometry_msgs::AccelWithCovariance::_covariance_type& resultingCovariance = presentMS.accel.covariance;
    cWiseAdd(transformedTwist0.covariance, twist1.covariance, resultingCovariance);
    cWiseDivide(resultingCovariance, (1 / (deltaTime * deltaTime)), resultingCovariance);

    return true;
}

bool incorporatePrecedingDataToMotionstate(const automated_driving_msgs::MotionState& precedingMS,
                                           automated_driving_msgs::MotionState& presentMS) {
    bool changed = false;
    // Keep Twist if value is valid, otherwise try to incorporate data from preceding motionState
    if (!checks::twistValid(presentMS)) {
        if (incorporatePrecedingPoseToTwist(precedingMS, presentMS)) {
            changed = true;
        }
    }

    // Keep Acceleration if value is valid, otherwise try to incorporate data from preceding motionState
    if (!checks::accelValid(presentMS)) {
        if (incorporatePrecedingTwistToAccel(precedingMS, presentMS)) {
            changed = true;
        }
    }
    return changed;
}


} // namespace util_automated_driving_msgs::computations

} // namespace util_automated_driving_msgs
