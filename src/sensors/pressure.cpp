/*
 * Author: Atte Niemi
 * Organisation: HYPED
 * Date: 30/5/21
 * Description:
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

#include <stdio.h>

#include "sensors/pressure.hpp"
#include "utils/io/adc.hpp"

namespace hyped {

using data::Data;
using data::PressureData;
using utils::io::ADC;
using hyped::utils::Logger;

namespace sensors {

Pressure::Pressure(utils::Logger& log, int pin)
     : pin_(pin),
       log_(log)
{}

void Pressure::run()
{
  ADC thepin(pin_);
  pressure_.pressure = 0;
  uint16_t raw_value = thepin.read();
  log_.DBG3("PRESSURE", "Raw Data: %d", raw_value);
  pressure_.pressure = scaleData(raw_value);
  log_.DBG3("PRESSURE", "Scaled Data: %d", pressure_.pressure);
  pressure_.operational = true;
}

int Pressure::scaleData(uint16_t raw_value)
{
  // convert to bar
  // assuming this is a conversion to volts (should that division be 4096?)
  double pressure = static_cast<double>(raw_value) / 4095;
  // naive maths: 0-5 volt to 0-10 bar equals multiply by 2
  pressure = 2 * pressure;
  // truncate down - should work since we are considering an integer-valued threshold
  return static_cast<int>(pressure);
}

int Pressure::getData()
{
  return pressure_.pressure;
}
}}  // namespace hyped::sensors
