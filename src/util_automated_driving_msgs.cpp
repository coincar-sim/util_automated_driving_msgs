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

bool frameIdsSet(const automated_driving_msgs::ObjectStateArray& objectStateArray, std::string& missingInformation) {
    bool frameIdsSet = true;
    missingInformation.clear();
    if (objectStateArray.header.frame_id.empty()) {
        missingInformation += "objectStateArray.header.frame_id is not set; ";
        frameIdsSet = false;
    }
    for (auto& objectState : objectStateArray.objects) {
        std::string objectPrefix = "objectStateArray.object[id=" + std::to_string(objectState.object_id) + "]: ";
        // Check objectState
        if (objectState.header.frame_id.empty()) {
            missingInformation += objectPrefix + "header.frame_id is not set; ";
            frameIdsSet = false;
        }
        // Check motion_state
        if (objectState.motion_state.header.frame_id.empty()) {
            missingInformation += objectPrefix + "motion_state.header.frame_id is not set; ";
            frameIdsSet = false;
        }
        if (objectState.motion_state.child_frame_id.empty()) {
            missingInformation += objectPrefix + "motion_state.child_frame_id is not set; ";
            frameIdsSet = false;
        }
        // Check motion_prediction
        if (!objectState.motion_prediction.trajectories.empty()) {
            if (objectState.motion_prediction.header.frame_id.empty()) {
                missingInformation += objectPrefix + "motion_prediction.header.frame_id is not set; ";
                frameIdsSet = false;
            }
            for (auto& trajectory : objectState.motion_prediction.trajectories) {
                std::string trajectoryPrefix =
                    objectPrefix + "motion_prediction.trajectories[id=" + std::to_string(trajectory.id) + "]: ";
                for (auto& motionState : trajectory.motion_states) {
                    if (motionState.header.frame_id.empty()) {
                        missingInformation +=
                            trajectoryPrefix +
                            "objectState.motion_prediction.trajectories motionState.header.frame_id is not set; ";
                        frameIdsSet = false;
                    }
                    if (motionState.child_frame_id.empty()) {
                        missingInformation +=
                            trajectoryPrefix +
                            "objectState.motion_prediction.trajectories motionStates.child_frame_id is not set; ";
                        frameIdsSet = false;
                    }
                }
            }
        } else {
            missingInformation +=
                objectPrefix + "objectState.motion_prediction.trajectories is empty, thus, omitting further checks";
        }
    }
    return frameIdsSet;
}

bool frameIdsValid(const automated_driving_msgs::ObjectStateArray& objectStateArray, std::string& missingInformation) {
    bool frameIdsValid = true;
    missingInformation.clear();
    std::string objectStateArrayHeaderFrameId = objectStateArray.header.frame_id;
    auto comparisionString([objectStateArrayHeaderFrameId](std::string currentFrameId) {
        return "(" + objectStateArrayHeaderFrameId + " != " + currentFrameId + "); ";
    });
    for (auto& objectState : objectStateArray.objects) {
        std::string objectPrefix = "objectStateArray.object[id=" + std::to_string(objectState.object_id) + "]: ";
        // Check objectState
        if (objectStateArrayHeaderFrameId != objectState.header.frame_id) {
            missingInformation +=
                objectPrefix + "header.frame_id is not valid " + comparisionString(objectState.header.frame_id);
            frameIdsValid = false;
        }
        // Check motion_state
        if (objectStateArrayHeaderFrameId != objectState.motion_state.header.frame_id) {
            missingInformation += objectPrefix + "motion_state.header.frame_id is not valid " +
                                  comparisionString(objectState.motion_state.header.frame_id);
            frameIdsValid = false;
        }
        // Check motion_prediction
        if (!objectState.motion_prediction.trajectories.empty()) {
            if (objectStateArrayHeaderFrameId != objectState.motion_prediction.header.frame_id) {
                missingInformation += objectPrefix + "motion_prediction.header.frame_id is not valid " +
                                      comparisionString(objectState.motion_prediction.header.frame_id);
                frameIdsValid = false;
            }
            for (auto& trajectory : objectState.motion_prediction.trajectories) {
                std::string trajectoryPrefix =
                    objectPrefix + "motion_prediction.trajectories[id=" + std::to_string(trajectory.id) + "]: ";
                for (auto& motionState : trajectory.motion_states) {
                    if (objectStateArrayHeaderFrameId != motionState.header.frame_id) {
                        missingInformation +=
                            trajectoryPrefix +
                            "objectState.motion_prediction.trajectories motionState.header.frame_id is not valid " +
                            comparisionString(motionState.header.frame_id);
                        frameIdsValid = false;
                    }
                }
            }
        } else {
            missingInformation +=
                objectPrefix + "objectState.motion_prediction.trajectories is empty, thus, omitting further checks";
        }
    }
    return frameIdsValid;
}

bool stampsWithinTimeRange(const automated_driving_msgs::ObjectStateArray& objectStateArray,
                           const double& timeRangeSecs,
                           std::string& missingInformation) {

    bool stampsValid = firstStampsAreEqual(objectStateArray, missingInformation);

    ros::Time earliestStamp{objectStateArray.header.stamp};
    ros::Time latestStamp{objectStateArray.header.stamp};

    auto updateStamps([&earliestStamp, &latestStamp](ros::Time currentStamp) {
        if (currentStamp < earliestStamp) {
            earliestStamp = currentStamp;
        } else if (currentStamp > latestStamp) {
            latestStamp = currentStamp;
        }
    });

    for (auto& objectState : objectStateArray.objects) {
        // Check objectState
        updateStamps(objectState.header.stamp);
        // Check motion_state
        updateStamps(objectState.motion_state.header.stamp);
        // Check motion_prediction
        if (!objectState.motion_prediction.trajectories.empty()) {
            updateStamps(objectState.motion_prediction.header.stamp);
            for (auto& trajectory : objectState.motion_prediction.trajectories) {
                for (auto& motionState : trajectory.motion_states) {
                    updateStamps(motionState.header.stamp);
                }
            }
        }
    }
    if (earliestStamp == ros::Time{0}) {
        missingInformation += "Earliest time stamp is: " + std::to_string(earliestStamp.toSec()) +
                              ". This should not happen, probably missing stamp initialization.";
        stampsValid = false;
    } else {
        if (latestStamp - earliestStamp < ros::Duration(timeRangeSecs)) {
            stampsValid = true;
        } else {
            stampsValid = false;
            missingInformation += "Earliest time stamp: " + std::to_string(earliestStamp.toSec()) +
                                  ", latest time stamp: " + std::to_string(latestStamp.toSec());
        }
    }
    return stampsValid;
}

bool firstStampsAreEqual(const automated_driving_msgs::ObjectStateArray& objectStateArray,
                         std::string& missingInformation) {
    bool stampsValid = true;
    const ros::Time& refStamp = objectStateArray.header.stamp;
    missingInformation.clear();

    for (auto& objectState : objectStateArray.objects) {

        if (objectState.header.stamp != refStamp) {
            stampsValid = false;
            missingInformation += "Stamp of object[id=" + std::to_string(objectState.object_id) + "] is inconsistent. ";
        }

        if (objectState.motion_state.header.stamp != refStamp) {
            stampsValid = false;
            missingInformation +=
                "Stamp of object[id=" + std::to_string(objectState.object_id) + "].motion_state is inconsistent. ";
        }

        if (!objectState.motion_prediction.trajectories.empty()) {

            if (objectState.motion_prediction.header.stamp != refStamp) {
                stampsValid = false;
                missingInformation += "Stamp of object[id=" + std::to_string(objectState.object_id) +
                                      "].motion_prediction is inconsistent. ";
            }

            for (auto& trajectory : objectState.motion_prediction.trajectories) {

                if (trajectory.motion_states.front().header.stamp != refStamp) {
                    stampsValid = false;
                    missingInformation += "Stamp of object[id=" + std::to_string(objectState.object_id) +
                                          "].motion_prediction.trajectories[id=" + std::to_string(trajectory.id) +
                                          "] is inconsistent. ";
                }
            }
        }
    }
    return stampsValid;
}


bool predictionStampsSynchronized(const automated_driving_msgs::ObjectStateArray& objectStateArray,
                                  const int64_t& predictionTimeStepMilliseconds,
                                  const double& predictionHorizonSeconds,
                                  std::string& missingInformation) {
    bool synced{true};
    const int64_t predictionTimeStepNanoseconds = int64_t(predictionTimeStepMilliseconds) * 1000000;
    const size_t predictionSteps =
        size_t(1000. / double(predictionTimeStepMilliseconds) * predictionHorizonSeconds) + 1;
    missingInformation.clear();

    // Check if starting point is the same
    synced = firstStampsAreEqual(objectStateArray, missingInformation);

    for (const automated_driving_msgs::ObjectState& object : objectStateArray.objects) {
        for (const automated_driving_msgs::Trajectory& traj : object.motion_prediction.trajectories) {
            if (traj.motion_states.size() != predictionSteps) {
                missingInformation += "Object[id=" + std::to_string(object.object_id) + "] Trajectory[id=" +
                                      std::to_string(traj.id) + "] has wrong number of motion_states. Expected " +
                                      std::to_string(predictionSteps) + " but found " +
                                      std::to_string(traj.motion_states.size()) + ". Moving to next trajectory;  ";
                synced = false;
                continue;
            }
            for (size_t it_ms{0}; it_ms < traj.motion_states.size(); it_ms++) {
                const automated_driving_msgs::MotionState& mS = traj.motion_states.at(it_ms);
                // Check if timestamps coincide with prediction time steps
                int64_t predictionDeltaTimeNanoseconds = int(it_ms) * predictionTimeStepNanoseconds;
                ros::Time predictionTimeExpected =
                    objectStateArray.header.stamp + ros::Duration().fromNSec(predictionDeltaTimeNanoseconds);

                if (predictionTimeExpected != mS.header.stamp) {
                    missingInformation +=
                        "Object[id=" + std::to_string(object.object_id) + "] Trajectory[id=" + std::to_string(traj.id) +
                        "] MotionState[#=" + std::to_string(it_ms) + "]=" + std::to_string(mS.header.stamp.sec) + "s" +
                        std::to_string(mS.header.stamp.nsec) + "ns " +
                        "deviates from policy start+n*predictionTimeStep! Expected: " +
                        std::to_string(predictionTimeExpected.sec) + "s" + std::to_string(predictionTimeExpected.nsec) +
                        "ns." + "Moving to next trajectory;  ";
                    synced = false;
                    break;
                }
            }
        }
    }
    if (!synced) {
        missingInformation += "(start=" + std::to_string(objectStateArray.header.stamp.sec) + "s" +
                              std::to_string(objectStateArray.header.stamp.nsec) + "ns, " + "step=" +
                              std::to_string(predictionTimeStepNanoseconds) + "ns)";
    }
    return synced;
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


void getInterpolationIndexAndScale(const automated_driving_msgs::Trajectory& traj,
                                   const ros::Time& interpolationTimestamp,
                                   const size_t& searchStartIndex,
                                   size_t& index,
                                   double& scale,
                                   bool& valid,
                                   std::string& errorMsg) {

    // Check if we have enough motion states
    if (traj.motion_states.size() < 2) {
        valid = false;
        errorMsg = "Not enough motion states available (need at least 2)";
        return;
    }

    if (searchStartIndex > traj.motion_states.size() - 2) {
        valid = false;
        errorMsg = "searchStartIndex too large";
        return;
    }

    double tInterpol = interpolationTimestamp.toSec();

    // Check if interpolationTimestamp is valid
    if (interpolationTimestamp < traj.motion_states.at(searchStartIndex).header.stamp) {
        errorMsg = "interpolationTimestamp out of range: smaller than "
                   "traj.motion_states.at(searchStartIndex).header.stamp; interpolationTimestamp=" +
                   std::to_string(interpolationTimestamp.toSec()) + ", at searchStartIndex=" +
                   std::to_string(traj.motion_states.at(searchStartIndex).header.stamp.toSec());
        valid = false;
        return;
    }

    if (interpolationTimestamp > traj.motion_states.back().header.stamp) {
        errorMsg = "interpolationTimestamp out of range: bigger than traj.motion_states.back().header.stamp; "
                   "interpolationTimestamp=" +
                   std::to_string(interpolationTimestamp.toSec()) + ", trajEndTime=" +
                   std::to_string(traj.motion_states.back().header.stamp.toSec());
        valid = false;
        return;
    }

    for (size_t i = searchStartIndex; i < traj.motion_states.size() - 1; i++) {
        double t0 = traj.motion_states.at(i).header.stamp.toSec();
        double t1 = traj.motion_states.at(i + 1).header.stamp.toSec();

        if (t0 <= tInterpol && tInterpol <= t1) {
            scale = double(tInterpol - t0) / double(t1 - t0);
            index = i;
            valid = true;
            return;
        }
    }

    errorMsg = "Pedantic: This should not happen!";
    valid = false;
    return;
}


void interpolateAlongTrajectory(const automated_driving_msgs::Trajectory& traj,
                                const ros::Time& interpolationTimestamp,
                                const size_t& searchStartIndex,
                                automated_driving_msgs::MotionState& interpolatedMotionState,
                                size_t& index,
                                bool& valid,
                                std::string& errorMsg) {

    double scale;

    getInterpolationIndexAndScale(traj, interpolationTimestamp, searchStartIndex, index, scale, valid, errorMsg);

    if (!valid) {
        return;
    }

    geometry_msgs::Pose p0 = traj.motion_states.at(index).pose.pose;
    geometry_msgs::Pose p1 = traj.motion_states.at(index + 1).pose.pose;

    if (util_geometry_msgs::checks::containsNANs(p0) || util_geometry_msgs::checks::containsNANs(p1)) {
        valid = false;
        errorMsg = "poses to interpolate between contain NANs";
        return;
    }

    interpolatedMotionState.pose.pose = util_geometry_msgs::computations::interpolateBetweenPoses(p0, p1, scale);

    interpolatedMotionState.twist.twist.linear.x = traj.motion_states.at(index).twist.twist.linear.x;

    interpolatedMotionState.header.stamp = interpolationTimestamp;
    interpolatedMotionState.header.frame_id = traj.motion_states.at(0).header.frame_id;
}


std::vector<std_msgs::Header> generateSynchronizedHeaders(const std_msgs::Header& firstHeader,
                                                          const int32_t& timeStepMilliseconds,
                                                          const double& predictionHorizonSeconds) {

    const int64_t timeStepNanoseconds = int64_t(timeStepMilliseconds) * 1000000;
    const int predictionSteps = int(1000. / double(timeStepMilliseconds) * predictionHorizonSeconds) + 1;

    std::vector<std_msgs::Header> headers;
    headers.resize(predictionSteps);

    for (int i = 0; i < predictionSteps; i++) {
        headers[i] = firstHeader;
        headers[i].stamp += ros::Duration().fromNSec(i * timeStepNanoseconds);
    }

    return headers;
}


void synchronizePredictionTimestamps(const automated_driving_msgs::Trajectory& unsyncedTrajectory,
                                     const int32_t& timeStepMilliseconds,
                                     const double& predictionHorizonSeconds,
                                     automated_driving_msgs::Trajectory& syncedTrajectory,
                                     bool& valid,
                                     std::string& errorMsg) {

    syncedTrajectory.id = unsyncedTrajectory.id;
    syncedTrajectory.probability = unsyncedTrajectory.probability;
    syncedTrajectory.motion_states.clear();

    auto headers = generateSynchronizedHeaders(unsyncedTrajectory.motion_states.front().header,
                                               timeStepMilliseconds,
                                               predictionHorizonSeconds);

    size_t indexOfPreviousInterpolation{0};
    for (size_t i{0}; i < headers.size(); i++) {
        automated_driving_msgs::MotionState motionState;
        const ros::Time interpolationTimestamp = headers.at(i).stamp;
        util_automated_driving_msgs::computations::interpolateAlongTrajectory(unsyncedTrajectory,
                                                                              interpolationTimestamp,
                                                                              indexOfPreviousInterpolation,
                                                                              motionState,
                                                                              indexOfPreviousInterpolation,
                                                                              valid,
                                                                              errorMsg);
        if (valid) {
            syncedTrajectory.motion_states.push_back(motionState);
        } else {
            errorMsg += " (in step " + std::to_string(i) + " of " + std::to_string(headers.size()) + ")";
            return;
        }
    }
}

void synchronizePredictionTimestamps(const automated_driving_msgs::ObjectStateArray& unsyncedObjectStates,
                                     const int32_t& timeStepMilliseconds,
                                     const double& predictionHorizonSeconds,
                                     automated_driving_msgs::ObjectStateArray& syncedObjectStates,
                                     bool& valid,
                                     std::string& errorMsg) {

    valid = true;
    if (!checks::firstStampsAreEqual(unsyncedObjectStates, errorMsg)) {
        valid = false;
        return;
    }


    syncedObjectStates.header = unsyncedObjectStates.header;
    syncedObjectStates.objects.clear();

    for (const auto& unsyncedOS : unsyncedObjectStates.objects) {
        automated_driving_msgs::ObjectState syncedObjectState{unsyncedOS};
        syncedObjectState.motion_prediction.trajectories.clear();
        //        if (unsyncedOS.motion_prediction.trajectories.size() < 1) {
        //            valid = false;
        //            errorMsg += "Object[id=" + std::to_string(unsyncedOS.object_id) + "] does not have a prediction.
        //            ";
        //        }

        for (size_t it_traj{0}; it_traj < unsyncedOS.motion_prediction.trajectories.size(); it_traj++) {
            automated_driving_msgs::Trajectory syncedTraj;
            bool validTemp{false};
            synchronizePredictionTimestamps(unsyncedOS.motion_prediction.trajectories.at(it_traj),
                                            timeStepMilliseconds,
                                            predictionHorizonSeconds,
                                            syncedTraj,
                                            validTemp,
                                            errorMsg);
            if (validTemp) {
                syncedObjectState.motion_prediction.trajectories.push_back(syncedTraj);
            } else {
                valid = false;
                errorMsg += " (in traj_id=" + std::to_string(unsyncedOS.motion_prediction.trajectories.at(it_traj).id) +
                            " of obj_id=" + std::to_string(unsyncedOS.object_id) + ")";
            }
        }
        syncedObjectStates.objects.push_back(syncedObjectState);
    }
}


} // namespace util_automated_driving_msgs::computations

} // namespace util_automated_driving_msgs
