/*
 * Authors: M. Kristien and Gregory Dayao
 * Organisation: HYPED
 * Date: 12. April 2018
 * Description:
 *
 *    Copyright 2018 HYPED
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

#include "sensors/bms.hpp"

#include "utils/logger.hpp"
#include "utils/timer.hpp"

namespace hyped {

namespace sensors {

std::vector<uint8_t> BMS::existing_ids_;    // NOLINT [build/include_what_you_use]
int16_t BMS::current_ = 0;
BMS::BMS(uint8_t id, Logger& log)
    : Thread(log),
      data_({}),
      id_(id),
      id_base_(bms::kIdBase + (bms::kIdIncrement * id_)),
      last_update_time_(0),
      can_(Can::getInstance()),
      running_(false)
{
  ASSERT(id < data::Batteries::kNumLPBatteries);
  // verify this BMS unit has not been instantiated
  for (uint8_t i : existing_ids_) {
    if (id == i) {
      log_.ERR("BMS", "BMS %d already exists, duplicate unit instantiation", id);
      return;
    }
  }
  existing_ids_.push_back(id);

  // tell CAN about yourself
  can_.registerProcessor(this);
  can_.start();

  running_ = true;
}

BMS::~BMS()
{
  running_ = false;
  join();
}

void BMS::request()
{
  // send request CanFrame
  utils::io::can::Frame message;
  message.id        = bms::kIdBase + (bms::kIdIncrement * id_);
  message.extended  = true;
  message.len       = 2;
  message.data[0]   = 0;
  message.data[1]   = 0;

  int sent = can_.send(message);
  if (sent) {
    log_.DBG1("BMS", "module %u: request message sent", id_);
  } else {
    log_.ERR("BMS", "module %u error: request message not sent", id_);
  }
}

void BMS::run()
{
  log_.INFO("BMS", "module %u: starting BMS", id_);
  while (running_) {
    request();
    sleep(bms::kPeriod);
  }
  log_.INFO("BMS", "module %u: stopped BMS", id_);
}

bool BMS::hasId(uint32_t id, bool extended)
{
  if (!extended) return false;  // this BMS only understands extended IDs

  // LP BMS CAN messages
  if (id_base_ <= id && id < id_base_ + bms::kIdSize) return true;

  // LP current CAN message
  if (id == 0x28) return true;

  return false;
}

void BMS::processNewData(utils::io::can::Frame& message)
{
  log_.DBG1("BMS", "module %u: received CAN message with id %d", id_, message.id);

  // check current CAN message
  if (message.id == 0x28) {
    if (message.len < 3) {
      log_.ERR("BMS", "module %u: current reading not enough data", id_);
      return;
    }

    current_ = (message.data[1] << 8) | (message.data[2]);
    return;
  }

  log_.DBG2("BMS", "message data[0,1] %d %d", message.data[0], message.data[1]);
  uint8_t offset = message.id - (bms::kIdBase + (bms::kIdIncrement * id_));
  switch (offset) {
    case 0x1:   // cells 1-4
      for (int i = 0; i < 4; i++) {
        data_.voltage[i] = (message.data[2*i] << 8) | message.data[2*i + 1];
      }
      break;
    case 0x2:   // cells 5-7
      for (int i = 0; i < 3; i++) {
        data_.voltage[4 + i] = (message.data[2*i] << 8) | message.data[2*i + 1];
      }
      break;
    case 0x3:   // ignore, no cells connected
      break;
    case 0x4:   // temperature
      data_.temperature = message.data[0] - bms::Data::kTemperatureOffset;
      break;
    default:
      log_.ERR("BMS", "received invalid message, id %d, CANID %d, offset %d",
          id_, message.id, offset);
  }

  last_update_time_ = utils::Timer::getTimeMicros();
}

bool BMS::isOnline()
{
  // consider online if the data has been updated in the last second
  return (utils::Timer::getTimeMicros() - last_update_time_) < 1000000;
}

void BMS::getData(BatteryData* battery)
{
  battery->voltage = 0;
  for (uint16_t v: data_.voltage) battery->voltage += v;
  battery->voltage    /= 100;  // scale to dV from mV
  battery->average_temperature = data_.temperature;
  if (battery->average_temperature == -40) battery->average_temperature = 0;    // if temp offline
  battery->current     = current_ - 0x800000;  // offset provided by datasheet
  battery->current    /= 100;   // scale to dA from mA

  // not used, initialised to zero
  battery->low_temperature = 0;
  battery->high_temperature = 0;
  battery->low_voltage_cell = 0;
  battery->high_voltage_cell = 0;

  // charge calculation
  if (battery->voltage >= 252) {                                     // constant high
    battery->charge = 95;
  } else if (252 > battery->voltage && battery->voltage >= 210) {    // linear high
    battery->charge = static_cast<uint8_t>(std::round((battery->voltage - 198.8) * (25/14)));
  } else if (210 > battery->voltage && battery->voltage >= 207) {    // binomial low
    battery->charge = 15;
  } else if (207 > battery->voltage && battery->voltage >= 200) {    // binomial low
    battery->charge = 10;
  } else if (200 > battery->voltage && battery->voltage >= 189) {    // binomial low
    battery->charge = 5;
  } else {                                                           // constant low
    battery->charge = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// BMSHP
////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<uint16_t> BMSHP::existing_ids_;   // NOLINT [build/include_what_you_use]

BMSHP::BMSHP(uint16_t id, Logger& log)
    : log_(log),
//      can_id_(id*2 + bms::kHPBase),
      hcu_maxt_id_(id + bms::kHPHcuMaxtBase),
      hcu_info_id_(id + bms::kHPHcuInfoBase),
      hcu_max_id_(id + bms::kHPHcuMaxvBase)
      cell_id_(bms::kHPCellBase),
      local_data_ {},
      last_update_time_(0)
{
  // verify this BMSHP unit has not been instantiated
  for (uint16_t i : existing_ids_) {
    if (id == i) {
      log_.ERR("BMSHP", "BMSHP %d already exists, duplicate unit instantiation", id);
      return;
    }
  }
  existing_ids_.push_back(id);

  // tell CAN about yourself
  Can::getInstance().registerProcessor(this);
  Can::getInstance().start();
}

bool BMSHP::isOnline()
{
  // consider online if the data has been updated in the last second
  return (utils::Timer::getTimeMicros() - last_update_time_) < 1000000;
}

void BMSHP::getData(BatteryData* battery)
{
  *battery = local_data_;
}

bool BMSHP::hasId(uint32_t id, bool extended)
{
  //if is BMS_HCU_MAXT message
  if (message.id == hcu_maxt_id_) return true;
  //if is BMS_HCU_INFO message
  if (message.id == hcu_info_id_) return true;
  //if is BMS_HCU_MAX message
  if (message.id == hcu_max_id_) return true;
  //if is BMS_HCU_CELLV message
  if (((message.id << 16) >> 16) == cell_id_) { //check if ending with 50F3
    int index = static_cast<int>((message.id << 8)>>24) //index between 00 and 4F
    if (index >= 0 && index <=79) {
      return true;
  }
  return false;
}

void BMSHP::processNewData(utils::io::can::Frame& message)
{
  // HP temp message (BMS_HCU_MAXT)
  // [MaxTemp, MaxTemp, MaxTempNo, MinTempNo, CoolingCtl, HeatingCtl]
  if (message.id == hcu_maxt_id_) {   // C
    local_data_.low_temperature     = message.data[0];
    local_data_.high_temperature    = message.data[1];
  }
  log_.DBG2("BMSHP", "High Temp: %d, Low Temp: %d",
    local_data_.high_temperature,
    local_data_.low_temperature);

  // HP BMS_HCU_INFO
  // [BatVoltage(MSB), BatVoltage(LSB), BatVoltage(LSB), BatVoltage(LSB), BatSoc, ....]
  if (message.id == hcu_info_id_){
    local_data_.voltage     = (message.data[0] << 8) | message.data[1];           // V
    local_data_local_data_.current     = (message.data[2] << 8) | message.data[3];// V
    local_data_.charge      = (message.data[4]);                                  // %
  }

  // HP BMS_HCU_MAX
  // [MaxCellVolt(MSB), MaxCellVolt(LSB), MinCellVolt(MSB), MinCellVolt(LSB), ....]
  if (message.id == hcu_max_id_){
    local_data_.high_voltage_cell = ((message.data[0] << 8) | message.data[1]);   // mV
    local_data_.low_voltage_cell  = ((message.data[2] << 8) | message.data[3]);   // mV
  }
  last_update_time_ = utils::Timer::getTimeMicros();
  log_.DBG2("BMSHP", "received data Volt,Curr,Char,low_v,high_v: %u,%u,%u,%u,%u",
            local_data_.voltage,
            local_data_.current,
            local_data_.charge,
            local_data_.low_voltage_cell,
            local_data_.high_voltage_cell);


  // BMS_HCU_CELLT check each cell voltage
  // [CellVoltN+1, CellVoltN+2, CellVoltN+3, CellVoltN+4]
  // message.id between 0x180050F3 and 0x184F50F3
  if (((message.id << 16) >> 16) == cell_id_) { //check if ending with 50F3
    int index = static_cast<int>((message.id << 8)>>24) //index between 00 and 4F
      if (index >= 0 && index <=79) {
        local_data_.cell_voltage[index*4 + 0] = (message.data[0] << 8) | message.data[1]   //mv
        local_data_.cell_voltage[index*4 + 1] = (message.data[2] << 8) | message.data[3]   //mv
        local_data_.cell_voltage[index*4 + 2] = (message.data[4] << 8) | message.data[5]   //mv
        local_data_.cell_voltage[index*4 + 3] = (message.data[6] << 8) | message.data[7]   //mv
    }
    log_.DBG2("BMSHP", "Index: %u Cells voltage: %u,%u,%u,%u",
      index,
      local_data_.cell_voltage[index*4 + 0],
      local_data_.cell_voltage[index*4 + 1],
      local_data_.cell_voltage[index*4 + 2],
      local_data_.cell_voltage[index*4 + 3]);
  }

}
}}  // namespace hyped::sensors
