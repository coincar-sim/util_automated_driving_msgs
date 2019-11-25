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

#include <boost/make_shared.hpp>

using namespace util_automated_driving_msgs::conversions;

class UtilAutomatedDrivingMsgsConversions : public ::testing::Test {
protected:
    UtilAutomatedDrivingMsgsConversions() {
        automated_driving_msgs::ObjectState os;
        os.object_id = 1;
        osa_.objects.push_back(os);

        os.object_id = 2;
        osa_.objects.push_back(os);

        os.object_id = 3;
        osa_.objects.push_back(os);

        os.object_id = 4;
        osa_.objects.push_back(os);

        osaPtr_ = boost::make_shared<automated_driving_msgs::ObjectStateArray>(osa_);
        osaConstPtr_ = boost::make_shared<automated_driving_msgs::ObjectStateArray>(osa_);
    }
    automated_driving_msgs::ObjectStateArray osa_;
    automated_driving_msgs::ObjectStateArrayPtr osaPtr_;
    automated_driving_msgs::ObjectStateArrayConstPtr osaConstPtr_;
};

TEST_F(UtilAutomatedDrivingMsgsConversions, objectStateFromObjectStateArray) {
    bool foundAndUnique;
    automated_driving_msgs::ObjectState os = objectStateFromObjectStateArray(osaPtr_, 1, foundAndUnique);
    EXPECT_TRUE(foundAndUnique);
    EXPECT_EQ(1u, os.object_id);

    automated_driving_msgs::ObjectState osNotFound = objectStateFromObjectStateArray(osaPtr_, 5, foundAndUnique);
    EXPECT_FALSE(foundAndUnique);
}

TEST_F(UtilAutomatedDrivingMsgsConversions, removeObjectFromObjectStateArray) {
    automated_driving_msgs::ObjectStateArray osaNew = removeObjectFromObjectStateArray(osa_, 3);
    EXPECT_EQ(3u, osaNew.objects.size());
    EXPECT_EQ(1u, osaNew.objects.at(0).object_id);
    EXPECT_EQ(2u, osaNew.objects.at(1).object_id);
    EXPECT_EQ(4u, osaNew.objects.at(2).object_id);
}

TEST(UtilAutomatedDrivingMsgsConversionsNoFixture, convertObjectClassification) {
    auto os = boost::make_shared<automated_driving_msgs::ObjectState>();
    automated_driving_msgs::ClassWithProbability c1, c2;
    c1.classification = automated_driving_msgs::ObjectClassification::BICYCLE;
    c1.probability = 0.3f;
    c2.classification = automated_driving_msgs::ObjectClassification::MOTORBIKE;
    c2.probability = 0.7f;
    os->classification.classes_with_probabilities.emplace_back(c1);
    os->classification.classes_with_probabilities.emplace_back(c2);
    auto res = convertObjectClassification(os->classification);
    EXPECT_FLOAT_EQ(static_cast<float>(res.at(automated_driving_msgs::ObjectClassification::BICYCLE)), 0.3f);
    EXPECT_FLOAT_EQ(static_cast<float>(res.at(automated_driving_msgs::ObjectClassification::MOTORBIKE)), 0.7f);
    EXPECT_FLOAT_EQ(static_cast<float>(res.at(automated_driving_msgs::ObjectClassification::PEDESTRIAN)), 0.0f);
}
