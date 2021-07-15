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

std::vector<uint8_t> BMS::existing_ids_;  // NOLINT [build/include_what_you_use]
int16_t BMS::current_ = 0;
BMS::BMS(uint8_t id, Logger &log)
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
  //LP BMS ID TBD(need to set on the converter)
  //Assume id == 300 for now
  //request cell voltage
  utils::io::can::Frame message;
  message.id       = 300
  message.extended = true;
  message.len      = 2;
  message.data[0]  = 0xAA;
  message.data[1]  = 0x1C;
  crc_word = CRC16(message.data,2);
  message.data[2]  = crc_word <<4;
  message.data[3]  = (crc_word << 4)>>4;

  int sent = can_.send(message);
  if (sent) {
    log_.DBG1("BMS", "module %u: request cell voltage message sent", id_);
  } else {
    log_.ERR("BMS", "module %u error: request cell voltage message not sent", id_);
  }
  //temperature request message
  message.id       = 300
  message.extended = true;
  message.len      = 2;
  message.data[0]  = 0xAA;
  message.data[1]  =  0x1B;
  crc_word = CRC16(message.data,2);
  message.data[2]  = crc_word <<4;
  message.data[3]  = (crc_word << 4)>>4;
  int sent = can_.send(message);
  if (sent) {
    log_.DBG1("BMS", "module %u: request temperature message sent", id_);
  } else {
    log_.ERR("BMS", "module %u error: temperature voltage message not sent", id_);
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
  //LP BMS ID TBD(need to set on the converter)
  //Assume id == 300 for now
  if (message.id == 300){
    return true
  }
}

void BMS::processNewData(utils::io::can::Frame &message)
{
  log_.DBG1("BMS", "module %u: received CAN message with id %d", id_, message.id);

  if (messsage.data[1] == 0x1C){
    for (int i = 0; i < 16; i++) {
      data_.voltage[i] = (message.data[2 * i + 4] << 1) | message.data[2 * i + 5];
    }
  }
  if (messsage.data[1] == 0x1B){
    data_.temperature = (message.data[6] << 1) | message.data[7];
  }
  last_update_time_ = utils::Timer::getTimeMicros();
}

bool BMS::isOnline()
{
  // consider online if the data has been updated in the last second
  return (utils::Timer::getTimeMicros() - last_update_time_) < 1000000;
}

void BMS::getData(BatteryData *battery)
{
  battery->voltage = 0;
  for (uint16_t v : data_.voltage)
    battery->voltage += v;
  battery->voltage /= 100;  // scale to dV from mV
  battery->average_temperature = data_.temperature;
  if (battery->average_temperature == -40) battery->average_temperature = 0;  // if temp offline
  battery->current = current_ - 0x800000;  // offset provided by datasheet
  battery->current /= 100;                 // scale to dA from mA

  // not used, initialised to zero
  battery->low_temperature   = 0;
  battery->high_temperature  = 0;
  battery->low_voltage_cell  = 0;
  battery->high_voltage_cell = 0;

  // charge calculation
  if (battery->voltage >= 252) {  // constant high
    battery->charge = 95;
  } else if (252 > battery->voltage && battery->voltage >= 210) {  // linear high
    battery->charge = static_cast<uint8_t>(std::round((battery->voltage - 198.8) * (25 / 14)));
  } else if (210 > battery->voltage && battery->voltage >= 207) {  // binomial low
    battery->charge = 15;
  } else if (207 > battery->voltage && battery->voltage >= 200) {  // binomial low
    battery->charge = 10;
  } else if (200 > battery->voltage && battery->voltage >= 189) {  // binomial low
    battery->charge = 5;
  } else {  // constant low
    battery->charge = 0;
  }
}

uint16_t CRC16(const uint8_t* data, uint16_t length)
{
  uint8_t tmp;
  uint16_t crcWord = 0xFFFF;
  while (length--)
  {
    tmp = *data++ ^ crcWord;
    crcWord >>= 8;
    crcWord ^= kcrcTable[tmp];
  }
  return crcWord;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// BMSHP
////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<uint16_t> BMSHP::existing_ids_;  // NOLINT [build/include_what_you_use]

BMSHP::BMSHP(uint16_t id, Logger &log)
    : log_(log),
      hcu_info_id_(id + bms::kHPHcuInfoBase),
      hcu_max_id_(id + bms::kHPHcuMaxvBase),
      hcu_maxt_id_(id + bms::kHPHcuMaxtBase),
      cell_id_(bms::kHPCellBase),
      local_data_{},
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

void BMSHP::getData(BatteryData *battery)
{
  *battery = local_data_;
}

bool BMSHP::hasId(uint32_t id, bool extended)
{
  // if is BMS_HCU_MAXT message
  if (id == hcu_maxt_id_) return true;
  // if is BMS_HCU_INFO message
  if (id == hcu_info_id_) return true;
  // if is BMS_HCU_MAX message
  if (id == hcu_max_id_) return true;
  // if is BMS_HCU_CELLV message
  if (((id << 16) >> 16) == cell_id_) {            // check if ending with 50F3
    int index = static_cast<int>((id << 8) >> 24);  // index between 00 and 4F
      if (index >= 0 && index <= 79) return true;
  }
  return false;
}

void BMSHP::processNewData(utils::io::can::Frame &message)
{
  // HP temp message (BMS_HCU_MAXT)
  // [MaxTemp, MaxTemp, MaxTempNo, MinTempNo, CoolingCtl, HeatingCtl]
  if (message.id == hcu_maxt_id_) {  // C
    local_data_.low_temperature  = message.data[0];
    local_data_.high_temperature = message.data[1];
  }
  log_.DBG2("BMSHP", "High Temp: %d, Low Temp: %d", local_data_.high_temperature,
            local_data_.low_temperature);

  // HP BMS_HCU_INFO
  // [BatVoltage(MSB), BatVoltage(LSB), BatVoltage(LSB), BatVoltage(LSB), BatSoc, ....]
  if (message.id == hcu_info_id_) {
    local_data_.voltage = (message.data[0] << 8) | message.data[1];  // V
    local_data_.current = (message.data[2] << 8) | message.data[3];  // V
    local_data_.charge  = (message.data[4]);                         // %
  }

  // HP BMS_HCU_MAX
  // [MaxCellVolt(MSB), MaxCellVolt(LSB), MinCellVolt(MSB), MinCellVolt(LSB), ....]
  if (message.id == hcu_max_id_) {
    local_data_.high_voltage_cell = ((message.data[0] << 8) | message.data[1]);  // mV
    local_data_.low_voltage_cell  = ((message.data[2] << 8) | message.data[3]);  // mV
  }
  last_update_time_ = utils::Timer::getTimeMicros();
  log_.DBG2("BMSHP", "received data Volt,Curr,Char,low_v,high_v: %u,%u,%u,%u,%u",
            local_data_.voltage, local_data_.current, local_data_.charge,
            local_data_.low_voltage_cell, local_data_.high_voltage_cell);

  // BMS_HCU_CELLT check each cell voltage
  // [CellVoltN+1, CellVoltN+2, CellVoltN+3, CellVoltN+4]
  // message.id between 0x180050F3 and 0x184F50F3
  if (((message.id << 16) >> 16) == cell_id_) {             // check if ending with 50F3

    int index = static_cast<int>((message.id << 8) >> 24);  // index between 00 and 4F

    if (index >= 0 && index <= 79) {
      local_data_.cell_voltage[index * 4 + 0] = (message.data[0] << 8) | message.data[1];  // mv
      local_data_.cell_voltage[index * 4 + 1] = (message.data[2] << 8) | message.data[3];  // mv
      local_data_.cell_voltage[index * 4 + 2] = (message.data[4] << 8) | message.data[5];  // mv
      local_data_.cell_voltage[index * 4 + 3] = (message.data[6] << 8) | message.data[7];  // mv
    }
    log_.DBG2("BMSHP", "Index: %u Cells voltage: %u,%u,%u,%u", index,
              local_data_.cell_voltage[index * 4 + 0], local_data_.cell_voltage[index * 4 + 1],
              local_data_.cell_voltage[index * 4 + 2], local_data_.cell_voltage[index * 4 + 3]);
  }
}
}  // namespace sensors
}  // namespace hyped
