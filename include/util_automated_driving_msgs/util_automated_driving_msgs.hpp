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

#pragma once

#include <ros/console.h>

#include <boost/array.hpp>

#include <automated_driving_msgs/MotionState.h>
#include <automated_driving_msgs/ObjectState.h>
#include <automated_driving_msgs/ObjectStateArray.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <tf2_eigen/tf2_eigen.h>

#include <util_geometry_msgs/util_geometry_msgs.hpp>

namespace {

static const double QUATERNION_TOLERANCE = 0.1f;

/** Mapping from the classification of an object to the probability of this classification */
using ProbabilityMap = std::map<int, double>;

/** @brief Calculating the componentwise sum of boost::array
 * @param[in] The first Array
 * @param[in] The second Array
 * @param[out] Array with componentwise sum.
 * @return void */
template <typename _Scalar, std::size_t size>
inline void cWiseAdd(const boost::array<_Scalar, size>& array0,
                     const boost::array<_Scalar, size>& array1,
                     boost::array<_Scalar, size>& result) {
    std::transform(array0.begin(), array0.end(), array1.begin(), result.begin(), std::plus<_Scalar>());
    return;
}

/** @brief Calculating the componentwise division of boost::array
 * numerator[i] / denominator[i] = result[i]
 * @param[in] The first Array, numerator
 * @param[in] The second Array, denominator
 * @param[out] Array with componentwise division.
 * @return void */
template <typename _Scalar, std::size_t size>
inline void cWiseDivide(const boost::array<_Scalar, size>& numerator,
                        const boost::array<_Scalar, size>& denominator,
                        boost::array<_Scalar, size>& result) {
    if (std::any_of(denominator.begin(), denominator.end(), [](_Scalar i) { return i == 0; })) {
        throw std::runtime_error("Division by zero.");
    }
    std::transform(numerator.begin(), numerator.end(), denominator.begin(), result.begin(), std::divides<_Scalar>());
    return;
}

/** @brief Calculating the componentwise division of boost::array with constant denominator
 * numerator[i] / denominator = result[i]
 * @param[in] The first Array, numerator
 * @param[in] A scalar, denominator
 * @param[out] Array with componentwise division.
 * @return void */
template <typename _Scalar, std::size_t size>
inline void cWiseDivide(const boost::array<_Scalar, size>& numerator,
                        const _Scalar& denominator,
                        boost::array<_Scalar, size>& result) {
    if (denominator == 0) {
        throw std::runtime_error("Division by zero.");
    }
    std::transform(numerator.begin(),
                   numerator.end(),
                   result.begin(),
                   std::bind(std::divides<_Scalar>(), std::placeholders::_1, denominator));
    return;
}

/** @brief Calculating the componentwise difference of 3D Vectors. x/y/z component-by-component
 * FirstParamIn - SecondParamIn = ParamOut
 * @param[in] The first 3D Vector.
 * @param[in] The second 3D Vector that is subtracted from the first. Componentwise. Must be of same Type than first
 * Vector.
 * @param[out] 3D Vectors with componentwise difference.
 * @return void */
template <typename Tin, typename Tout>
inline void cWiseDiffXYZ(const Tin& xyzVector0, const Tin& xyzVector1, Tout& result) {
    result.x = xyzVector0.x - xyzVector1.x;
    result.y = xyzVector0.y - xyzVector1.y;
    result.z = xyzVector0.z - xyzVector1.z;
    return;
}

/** @brief Calculating the differential quotient of 3D Vectors. x/y/z component-by-component
 * (FirstParamIn - SecondParamIn) / DeltaTime = ParamOut
 * @param[in] The first/preceding (older) 3D Vector.
 * @param[in] The second/subsequent (newer) 3D Vector. Must be of same Type as first Vector.
 * @param[in] The time difference between the first and second vector.
 * @param[out] Differential quotient of 3D Vectors. x/y/z component-by-component.
 * @return void */
template <typename Tin, typename Tout>
inline void diffQuotientXYZ(const Tin& xyzVector0, const Tin& xyzVector1, const double deltaTime, Tout& result) {
    if (deltaTime == 0) {
        throw std::runtime_error("Division by zero.");
    }
    result.x = (xyzVector1.x - xyzVector0.x) / deltaTime;
    result.y = (xyzVector1.y - xyzVector0.y) / deltaTime;
    result.z = (xyzVector1.z - xyzVector0.z) / deltaTime;
    return;
}

/** @brief Calculating the "differential quotient of rotation"
 * by taking the rotational difference between the orientations (as Quaternion),
 * converting to RollPitchYaw(RPY) representation
 * and dividing the RPY values by deltaTime.
 * Keep in mind: The angular velocity is not unambiguous assignable when calculated as difference from two
 * orientations
 * only.
 * @param[in] The first/preceding (older) orientation
 * @param[in] The second/subsequent (newer) orientation
 * @param[in] The time difference between the first and second orientation
 * @param[out] RPY-Representation of the rotational difference divided by the time difference.
 * @return void */
inline void diffQuotientOrientation(const geometry_msgs::Pose::_orientation_type& orientation0,
                                    const geometry_msgs::Pose::_orientation_type& orientation1,
                                    const double deltaTime,
                                    geometry_msgs::Twist::_angular_type& result) {

    Eigen::Quaternion<double> orientation0EQ, orientation1EQ;
    tf2::fromMsg(orientation0, orientation0EQ);
    tf2::fromMsg(orientation1, orientation1EQ);
    // util_geometry_msgs::conversions::fromMsg(orientation0, orientation0EQ);
    // util_geometry_msgs::conversions::fromMsg(orientation1, orientation1EQ);

    // calculate rotational difference
    Eigen::Quaternion<double> rot0to1EQ = (orientation0EQ.inverse() * orientation1EQ);
    rot0to1EQ.normalize();

    // There is no direct transformation to RPY (Euler X-Y-Z) therefore a detour via Matrix3x3 is used
    Eigen::Vector3d rpy = rot0to1EQ.toRotationMatrix().eulerAngles(0, 1, 2);

    rpy = rpy / deltaTime;
    // tf2::fromMsg(rpy, result);
    result = util_geometry_msgs::conversions::toMsg<geometry_msgs::Vector3>(rpy);
}
} // namespace

namespace util_automated_driving_msgs {

namespace conversions {

/** @brief Searches for an ObjectState defined by its object_id in an ObjectStateArray.
 * @param[in] The ObjectStateArray in which the ObjectState is searched.
 * @param[in] The object_id by which the searched ObjectState is defined.
 * @param[out] IndicatorValue: True if the ObjectState is found once (and only once) in the ObjectStateArray, false
 * otherwise.
 * @return the ObjectState from the ObjectStateArray defined by the object_id*/
automated_driving_msgs::ObjectState objectStateFromObjectStateArray(
    const automated_driving_msgs::ObjectStateArray& inputObjectArray, uint32_t objectId, bool& foundAndUnique);

/** @brief Searches for an ObjectState defined by its object_id in an ObjectStateArray.
 * @param[in] A boost::shared_pointer to the ObjectStateArray in which the ObjectState is searched.
 * @param[in] The object_id by which the searched ObjectState is defined.
 * @return the ObjectState from the ObjectStateArray defined by the object_id*/
automated_driving_msgs::ObjectState objectStateFromObjectStateArray(
    boost::shared_ptr<const automated_driving_msgs::ObjectStateArray> inputObjectArray,
    uint32_t objectId,
    bool& foundAndUnique);

/** @brief Searches for an ObjectState defined by its object_id and deletes it from an ObjectStateArray.
 * @param[in] The ObjectStateArray in which the ObjectState will be removed. !This ObjectStateArray is modified!
 * @param[in] The object_id by which the searched ObjectState is defined.
 * @return the ObjectStateArray with the removed ObjectState defined by the object_id*/
automated_driving_msgs::ObjectStateArray removeObjectFromObjectStateArray(
    const automated_driving_msgs::ObjectStateArray& inputObjectArray, uint32_t objectId);

/**
 * @brief convert
 * @param[in] classification the ObjectClassification to be converted.
 * @return a ProbabilityMap which map the classification to a probability. All available classes are set.
 */
ProbabilityMap convertObjectClassification(const automated_driving_msgs::ObjectClassification& classification);
} // namespace conversions

namespace checks {

/** @brief Checks if the entries for the position in a motionState are marked as reliable.
 * This is done by checking if the diagonal values in the covariance matrix for the position are >= 0
 * If you want to mark entries unreliable it is advisable to set the diagonal elements of the covariance matrix to
 * "-1".
 * @param[in] motionState to be checked.
 * @return true if the diagonal elements of the covariance matrix for the position are >=0, false otherwise.*/
bool positionValid(const automated_driving_msgs::MotionState& motionState);

/** @brief Checks if the entries for the orientation in a motionState are marked as reliable.
 * This is done by checking if the diagonal values in the covariance matrix for the orientation are >= 0
 * If you want to mark entries unreliable it is advisable to set the diagonal elements of the covariance matrix to
 * "-1".
 * @param[in] motionState to be checked.
 * @return true if the diagonal elements of the covariance matrix for the orientation are >=0, false otherwise.*/
bool orientationValid(const automated_driving_msgs::MotionState& motionState);

/** @brief Checks if pose in a motionState is valid.
 * This is done by checking if the position, the orientation and the related covariance matrix is valid.
 * @param[in] motionState to be checked.
 * @return true if the pose is valid, false otherwise.*/
bool poseValid(const automated_driving_msgs::MotionState& motionState);

/** @brief Checks if the entries for the linear twist in a motionState are marked as reliable.
 * This is done by checking if the diagonal values in the covariance matrix for the linear twist are >= 0
 * If you want to mark entries unreliable it is advisable to set the diagonal elements of the covariance matrix to
 * "-1".
 * @param[in] motionState to be checked.
 * @return true if the diagonal elements of the covariance matrix for the linear twist are >=0, false otherwise.*/
bool linearTwistValid(const automated_driving_msgs::MotionState& motionState);

/** @brief Checks if the entries for the angular twist in a motionState are marked as reliable.
 * This is done by checking if the diagonal values in the covariance matrix for the angular twist are >= 0
 * If you want to mark entries unreliable it is advisable to set the diagonal elements of the covariance matrix to
 * "-1".
 * @param[in] motionState to be checked.
 * @return true if the diagonal elements of the covariance matrix for the angular twist are >=0, false otherwise.*/
bool angularTwistValid(const automated_driving_msgs::MotionState& motionState);

/** @brief Checks if twist in a motionState is valid.
 * This is done by checking if the linear twist, the angular twist and the related covariance matrix is valid.
 * @param[in] motionState to be checked.
 * @return true if the twist is valid, false otherwise.*/
bool twistValid(const automated_driving_msgs::MotionState& motionState);

/** @brief Checks if the entries for the linear acceleration in a motionState are marked as reliable.
 * This is done by checking if the diagonal values in the covariance matrix for the linear acceleration are >= 0
 * If you want to mark entries unreliable it is advisable to set the diagonal elements of the covariance matrix to
 * "-1".
 * @param[in] motionState to be checked.
 * @return true if the diagonal elements of the covariance matrix for the linear acceleration are >=0, false
 * otherwise.*/
bool linearAccelValid(const automated_driving_msgs::MotionState& motionState);

/** @brief Checks if the entries for the angular acceleration in a motionState are marked as reliable.
 * This is done by checking if the diagonal values in the covariance matrix for the angular acceleration are >= 0
 * If you want to mark entries unreliable it is advisable to set the diagonal elements of the covariance matrix to
 * "-1".
 * @param[in] motionState to be checked.
 * @return true if the diagonal elements of the covariance matrix for the angular acceleration are >=0, false
 * otherwise.*/
bool angularAccelValid(const automated_driving_msgs::MotionState& motionState);

/** @brief Checks if accel in a motionState is valid.
 * This is done by checking if the linear acceleration, the angular acceleration and the related covariance matrix
 * is
 * valid.
 * @param[in] motionState to be checked.
 * @return true if the accel is valid, false otherwise.*/
bool accelValid(const automated_driving_msgs::MotionState& motionState);

/**
 * @brief frameIdsSet Checks if all frameIds in an objectStateArray are set (non-empty).
 * @param objectStateArray objectStateArray to be checked.
 * @param missingInformation String containing information about which frameIds are missing.
 * @return true if all frameIds are set, false otherwise. */
bool frameIdsSet(const automated_driving_msgs::ObjectStateArray& objectStateArray, std::string& missingInformation);

/**
 * @brief frameIdsValid Checks if all frameIds in an objectStateArray coincide.
 * @param objectStateArray objectStateArray to be checked.
 * @param missingInformation String containing information about which frameIds are missing.
 * @return true if all frameIds are valid, i.e. coincide, false otherwise. */
bool frameIdsValid(const automated_driving_msgs::ObjectStateArray& objectStateArray, std::string& missingInformation);

/**
 * @brief firstStampsAreEqual Checks if the first stamp of every submessage is the same.
 * @param objectStateArray objectStateArray to be checked.
 * @param missingInformation String containing information about which stamps are inconsistent.
 * @return true if all first stamps are the same, i.e. coincide, false otherwise.
 */
bool firstStampsAreEqual(const automated_driving_msgs::ObjectStateArray& objectStateArray,
                         std::string& missingInformation);

/**
 * @brief stampdsValid Checks if all stamps are within a windows of timeRangeSecs seconds.
 * @param objectStateArray objectStateArray to be checked.
 * @param timeRangeSecs allowed time range in seconds.
 * @param missingInformation String containing information about why stamps are not within rangeg.
 * @return true if all stampds are valid, i.e. within a range of 60 seconds, false otherwise. */
bool stampsWithinTimeRange(const automated_driving_msgs::ObjectStateArray& objectStateArray,
                           const double& timeRangeSecs,
                           std::string& missingInformation);

/**
 * @brief predictionSynchroniyzed Checks if of all time stamps of all predictions of objects are equal.
 * @param objectStateArray objectStateArray to be checked.
 * @param predictionTimeStepMilliseconds desired time between two motion states within the predicted trajectory
 * @param missingInformation String containing information about which frameIds are missing.
 * @return true if all time stamps are syncronized, false otherwise.
 */
bool predictionStampsSynchronized(const automated_driving_msgs::ObjectStateArray& objectStateArray,
                                  const int64_t& predictionTimeStepMilliseconds,
                                  const double& predictionHorizonSeconds,
                                  std::string& missingInformation);


} // namespace checks

namespace computations {

/** @brief Incorporate the Data of a preceding MotionState to a more recent MotionState.
 * If the Twist or Acceleration of the more recent MotionState is unavailable or unreliable (checking the covariance
 * matrix)
 * the Data of the preceding MotionState is used to complement the data of the more recent MotionState
 * by using the differential quotient of the pose (the twist respective).
 * If no improvement of the data is possible the MotionState remains unchanged.
 * @param[in] The first/preceding (older) MotionState. This MotionState is never changed.
 * @param[in,out] The second (present, more recent, newer) MotionState. This MotionState is changed, complemented by
 * the
 * Data in the preceding Motionstate
 * @return true if the second MotionState has been changed, false if not or on failure. */
bool incorporatePrecedingDataToMotionstate(const automated_driving_msgs::MotionState& precedingMS,
                                           automated_driving_msgs::MotionState& presentMS);


/**
 * @brief getInterpolationIndexAndScale
 * @param traj Current trajectory where interpolation is done.
 * @param interpolationTimestamp Time stamp that should be interpolated.
 * @param searchStartIndex Index where the previous result was found, makes search faster.
 * @param index Resulting index (of the motion state), that is right ahead of the interpolationTimestamp.
 * @param scale Resulting scale, telling relative time between index and the next motion state
 * @param valid Returns true, if all computations could be done without problems.
 * @param errorMsg Contains a error string, in case any errors occur.
 */
void getInterpolationIndexAndScale(const automated_driving_msgs::Trajectory& traj,
                                   const ros::Time& interpolationTimestamp,
                                   const size_t& searchStartIndex,
                                   size_t& index,
                                   double& scale,
                                   bool& valid,
                                   std::string& errorMsg);

/**
 * @brief interpolateAlongTrajectory !!assuming constant velocity!!
 * @param traj Current trajectory where interpolation is done.
 * @param interpolationTimestamp Time stamp that should be interpolated.
 * @param searchStartIndex Index where the previous result was found, makes search faster.
 * @param interpolatedMotionState Resulting interpolated motion state.
 * @param index Index where the result was found, serves as searchStartIndex for the next interpolation.
 * @param valid Returns true, if all computations could be done without problems.
 * @param errorMsg Contains a error string, in case any errors occur.
 */
void interpolateAlongTrajectory(const automated_driving_msgs::Trajectory& traj,
                                const ros::Time& interpolationTimestamp,
                                const size_t& searchStartIndex,
                                automated_driving_msgs::MotionState& interpolatedMotionState,
                                size_t& index,
                                bool& valid,
                                std::string& errorMsg);

/**
 * @brief generateSynchronizedHeaders
 * @param first_header
 * @param timeStepMilliseconds
 * @param predictionHorizonSeconds
 * @return
 */
std::vector<std_msgs::Header> generateSynchronizedHeaders(const std_msgs::Header& firstHeader,
                                                          const int32_t& timeStepMilliseconds,
                                                          const double& predictionHorizonSeconds);


/**
 * @brief synchronizePredictionTimestamps !!assuming constant velocity!!
 * @param unsyncedTrajectory
 * @param timeStepMilliseconds
 * @param predictionHorizonSeconds
 * @param syncedTrajectory
 * @param valid
 * @param errorMsg
 */
void synchronizePredictionTimestamps(const automated_driving_msgs::Trajectory& unsyncedTrajectory,
                                     const int32_t& timeStepMilliseconds,
                                     const double& predictionHorizonSeconds,
                                     automated_driving_msgs::Trajectory& syncedTrajectory,
                                     bool& valid,
                                     std::string& errorMsg);
/**
 * @brief synchronizePredictionTimestamps !!assuming constant velocity!!
 * @param unsyncedObjectStates
 * @param timeStepMilliseconds
 * @param predictionHorizonSeconds
 * @param syncedObjectStates
 * @param valid
 * @param errorMsg
 */
void synchronizePredictionTimestamps(const automated_driving_msgs::ObjectStateArray& unsyncedObjectStates,
                                     const int32_t& timeStepMilliseconds,
                                     const double& predictionHorizonSeconds,
                                     automated_driving_msgs::ObjectStateArray& syncedObjectStates,
                                     bool& valid,
                                     std::string& errorMsg);


} // namespace computations

} // namespace util_automated_driving_msgs
