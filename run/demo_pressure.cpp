/*
 * Author: Atte Niemi
 * Organisation: HYPED
 * Date: 31/5/21
 * Description: Demo for the emergency brakes' internal pressure sensor
 *
 *    Copyright 2021 HYPED
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

#include "sensors/pressure.hpp"
#include "utils/logger.hpp"
#include "utils/config.hpp"
#include "utils/system.hpp"
#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"

using hyped::utils::Logger;
using hyped::utils::concurrent::Thread;
using namespace hyped::data;
using hyped::data::Sensors;
using hyped::sensors::Pressure;
using hyped::utils::System;
using hyped::utils::Config;

int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  System& sys_ = hyped::utils::System::getSystem();
  Logger& log = hyped::utils::System::getLogger();
  Pressure* pressure = new Pressure(log, sys_.config->sensors.pressure);

  while (true) {
    pressure->run();
    Thread::sleep(100);
  }

  return 0;
}
