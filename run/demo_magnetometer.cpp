/*
 * Author: Yeyao Liu, Gregory Dayao
 * Organisation: HYPED
 * Date: 30/1/20
 * Description: Demo for ICM-20948 sensor using imu_manager
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

#include "sensors/magnetometer.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"

using hyped::utils::Logger;
using hyped::utils::concurrent::Thread;
using namespace hyped::data;
using hyped::data::Sensors;
using hyped::sensors::Magnetometer;
using hyped::utils::System;

int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  Logger& log = hyped::utils::System::getLogger();
  Data& data = Data::getInstance();

  DataPoint<array<ImuData, Sensors::kNumImus>> data_array_;
  ImuData magData;
  ImuData accData;

  hyped::utils::io::SPI::getInstance().setClock(hyped::utils::io::SPI::Clock::k4MHz);
  Magnetometer* imu = new Magnetometer(log, 47, false);
  // Magnetometer mag(log, 47, false);

  StateMachine state_machine = data.getStateMachineData();
  state_machine.current_state = State::kAccelerating;   // change state for different accel values
  data.setStateMachineData(state_machine);

  while (true) {
    imu->getData(&accData);
    imu->getMagData(&magData);
    log.ERR("IMU", "Accelerometer x: %f m/s^2, y: %f m/s^2, z: %f m/s^2 Magnetometer x: %.0f, y: %.0f, z: %.0f", 
    accData.acc[0], accData.acc[1], accData.acc[2], magData.acc[0], magData.acc[1], magData.acc[2]);
    // log.ERR("IMU", "Mag x: %.2f, y: %.2f, z: %.2f", magData.acc[0], magData.acc[1], magData.acc[2]);
    Thread::sleep(100);
  }

  return 0;
}
