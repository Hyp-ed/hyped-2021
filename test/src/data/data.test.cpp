/*
 * Author: Pablo Morandé
 * Organisation: HYPED
 * Date: 13/11/2020
 * Description: Testing file for data.cpp
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *    except in compliance with the License. You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software distributed under
 *    the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *    either express or implied. See the License for the specific language governing permissions and
 *    limitations under the License.
 */
#include "gtest/gtest.h"
#include "utils/logger.hpp"
#include "data/data.hpp"
struct NavigationDataTest: public ::testing ::Test
{
    hyped::data::Data* _d;
    hyped::data::Navigation nav_data;
    void SetUp()
    {
        _d = &hyped::data::Data::getInstance();

        nav_data.acceleration = 0;
        nav_data.braking_distance = 0;
        nav_data.emergency_braking_distance = 0;
        nav_data.velocity =0;
        nav_data.module_status = hyped::data::ModuleStatus::kReady;
    }

    void TearDown() {}
};
TEST_F(NavigationDataTest, NavigationDataSetGetTest)
{
    _d->setNavigationData(nav_data);
    hyped::data::Navigation received_nav_data = _d->getNavigationData();

    ASSERT_EQ(received_nav_data.acceleration, nav_data.acceleration);
    ASSERT_EQ(received_nav_data.braking_distance, nav_data.braking_distance);
    ASSERT_EQ(received_nav_data.emergency_braking_distance, nav_data.emergency_braking_distance);
    ASSERT_EQ(received_nav_data.module_status, nav_data.module_status);
    ASSERT_EQ(received_nav_data.velocity, nav_data.velocity);
    ASSERT_EQ(received_nav_data.module_status, hyped::data::ModuleStatus::kReady);
}
struct TelemetryDataTest : public ::testing ::Test
{
  hyped::data::Data* _d;
  hyped::data::Telemetry telemetry_data;
  void SetUp()
  {
    _d = &hyped::data::Data::getInstance();
    telemetry_data.emergency_stop_command = true;
    telemetry_data.launch_command = true;
    telemetry_data.nominal_braking_command = true;
    telemetry_data.reset_command = true;
    telemetry_data.service_propulsion_go = true;
    telemetry_data.module_status = hyped::data::ModuleStatus::kReady;
  }
};
TEST_F(TelemetryDataTest, NavigationDataSetGetTest)
{
  _d->setTelemetryData(telemetry_data);
  hyped::data::Telemetry received_telemetry_data = _d->getTelemetryData();

  ASSERT_EQ(received_telemetry_data.emergency_stop_command, telemetry_data.emergency_stop_command);
  ASSERT_EQ(received_telemetry_data.launch_command, telemetry_data.launch_command);
  ASSERT_EQ(received_telemetry_data.nominal_braking_command,
  telemetry_data.nominal_braking_command);
  ASSERT_EQ(received_telemetry_data.service_propulsion_go, telemetry_data.service_propulsion_go);
  ASSERT_EQ(received_telemetry_data.module_status, telemetry_data.module_status);
}
TEST(dataOnlyOneInstance, InstanceTest)
{
  hyped::data::Data* _d = &hyped::data::Data::getInstance();
  hyped::data::Data* _d2 = &hyped::data::Data::getInstance();
  ASSERT_EQ(_d, _d2);
}
