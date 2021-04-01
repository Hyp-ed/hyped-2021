/*
 * Authors: Pablo Morandé
 * Organisation: HYPED
 * Date:
 * Description: testing for kalman filters in navigation
 *
 *    Copyright 2020 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */
#include <iostream>
#include "math.h"
#include "gtest/gtest.h"
#include "navigation/kalman_filter.hpp"
#include "utils/system.hpp"

namespace hyped {
  using utils::System;
namespace navigation
{

    struct KalmanInitialisation : public ::testing::Test
{
  void SetUp()
  {
  }
};
TEST_F(KalmanInitialisation, handlesInisialisationOfState)
{
  ASSERT_EQ(1, 1);
}
}
}
