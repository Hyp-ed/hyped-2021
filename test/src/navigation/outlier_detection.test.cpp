/*
 * Author: Suryansh Manocha
 * Organisation: HYPED
 * Date: 30/12/2020
 * Description: Tests for the navigation outlier detection algorithms
 *
 *  Copyright 2020 HYPED
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *  except in compliance with the License. You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software distributed under
 *  the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *  either express or implied. See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include "gtest/gtest.h"
#include "utils/logger.hpp"
#include "data/data.hpp"
#include "navigation/outlier_detection.hpp"
#include "navigation/navigation.hpp"

using hyped::navigation::OutlierDetection;
using hyped::navigation::Navigation;
using hyped::data::NavigationType;

// Generate random int given int bounds
int RandomInt(int min, int max)
{
  return rand()%(max-min + 1) + min;
}

/*
 * Struct used for test fixtures testing the functionality of outlier_detection.hpp
 * Constructor 
 * 
*/
struct OutlierDetectionFunc : public ::testing::Test {
    protected:
        OutlierDetection outDetect = OutlierDetection();

        void SetUp() {
            // TODO: Smth
        }

        void TearDown() {
            // TODO: Smth
        }
};

// Test the function dead_IMUS()
TEST_F(OutlierDetectionFunc, deadIMUTest) {
    Navigation::NavigationArray imus;
    int expectedDeadIMUs = RandomInt(0, imus.size());
    // Populate the imus array with random floats
    for (int i; i < imus.size(); i++){
        imus[i] = RandomInt(0, imus.size()) / 1.3;
    }
    // Populate the imus array with randomly allocated zero values
    for (int _; _ < expectedDeadIMUs; _++){
        imus[RandomInt(0, imus.size())] = 0;
    }
    outDetect.dataArray_ = imus;

    ASSERT_EQ(outDetect.deadIMUs(), expectedDeadIMUs);
}

// Test the function getMean() with basic values
TEST_F(OutlierDetectionFunc, getMeanBasicTest){
    outDetect.dataArray_ = {2.5, 2.5, 2.5, 2.5};
    NavigationType expected_mean = 2.5;

    ASSERT_EQ(outDetect.getMean(), expected_mean);
}

// Test the function getMean() with complex values
TEST_F(OutlierDetectionFunc, getMeanComplexTest){
    outDetect.dataArray_ = {2.75, 2.33, 1.22, 8.45};
    NavigationType expected_mean = 3.6875;

    ASSERT_EQ(outDetect.getMean(), expected_mean);
}

// Test the function getMedian()
// Where there are no dead IMUs and
// where dead_IMUS() != kNumImus / 2
TEST_F(OutlierDetectionFunc, getMedianBasicTest){
    outDetect.dataArray_ = {3.5, 2.5, 8.45, 5.5};
    // Sorted: {2.5, 3.5, 5.5, 8.45}
    NavigationType expected_median = 4.5;

    ASSERT_EQ(outDetect.getMedianAdjusted(), expected_median);
}

// Test the function getMedian()
// Where there are dead IMUs
// Where dead_IMUS() != kNumImus / 2
TEST_F(OutlierDetectionFunc, getMedianComplex1Test){
    outDetect.dataArray_ = {0.23, 0, 0.9, 0.68};
    // Array:           {0.23, 0.00, 0.90, 0.68, 0.00, 0.00}
    // filtered:        {0.23, 0.23, 0.90, 0.68, 0.68, 0.68}
    // Sorted:          {0.23, 0.23, 0.68, 0.68, 0.68, 0.90}
    // {0.23, 0.9, 0.68}

    // Array:           {0.23, 0.90, 0.00, 0.00, 0.00, 0.68}
    // filtered:        {0.23, 0.90, 0.90, 0.90, 0.90, 0.68}
    // Sorted:          {0.23, 0.68, 0.90, 0.90, 0.90, 0.90}
    NavigationType expected_median = 0.9;

    ASSERT_EQ(outDetect.getMedianAdjusted(), expected_median);
}

// Test the function getMedian()
// Where there are dead IMUs
// Where dead_IMUS() != kNumImus / 2
TEST_F(OutlierDetectionFunc, getMedianComplex2Test){
    outDetect.dataArray_ = {0.23, 0, 0, 0.68};
    NavigationType expected_median = 0.455;

    ASSERT_EQ(outDetect.getMedianAdjusted(), expected_median);
}