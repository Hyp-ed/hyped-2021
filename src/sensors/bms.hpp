/*
 * Authors: M. Kristien and Gregory Dayao
 * Organisation: HYPED
 * Date: 12. April 2018
 * Description:
 * Battery Management System abstraction. Each object corresponds to one low-powered BMS unit.
 * Each unit is identified by a unique ID, which is also used to infer IDs of CAN messages.
 *
 * Configuration and local data structure are encapsulated in namespace bms.
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

#ifndef SENSORS_BMS_HPP_
#define SENSORS_BMS_HPP_

#include <cstdint>
#include <vector>

#include "sensors/interface.hpp"
#include "utils/concurrent/thread.hpp"
#include "utils/io/can.hpp"
#include "utils/system.hpp"
#include "utils/utils.hpp"

namespace hyped {

// Forward declarations
namespace utils {
class Logger;
}
namespace utils {
namespace io {
class Can;
}
}  // namespace utils
namespace utils {
namespace io {
class CanProccesor;
}
}  // namespace utils
namespace utils {
namespace io {
namespace can {
struct Frame;
}
}  // namespace io
}  // namespace utils

namespace sensors {

using data::Data;
using utils::Logger;
using utils::concurrent::Thread;
using utils::io::Can;
using utils::io::CanProccesor;

namespace bms {
// how often shall request messages be sent
constexpr uint32_t kFreq   = 4;             // in Hz
constexpr uint32_t kPeriod = 1000 / kFreq;  // in milliseconds

// what is the CAN ID space for BMS units
constexpr uint16_t kIdBase      = 300;  // Base for ids of CAN messages related to BMS
constexpr uint16_t kIdIncrement = 10;   // increment of base dependent on id_
constexpr uint16_t kIdSize      = 5;    // size of id-space of BMS-CAN messages
/**
 * Bases of IDs of CAN messagese for a BMS unit are calculated as follows:
 * base = kIdBase + (kIdIncrement * id_)
 */

/** OLD
 * LP: Notches:    0        1        2
 *     ID:      301-304, 311-314, 321-324
 *     Hex:     12D-130, 137-13A, 141-144
 * HP: ID:      1712-13, 1714-15
 *     Hex:     6B0-6B1, 6B2-6B3
 * Therm ID:   406451072  406451073
 *       Hex:  0x1839F380 0x1839F381
 */

/** NEW TBD
 * LP: Notches:    0        1        2
 *     ID:        ???
 *     Hex:       ???
 * HP: ID:      384-392, ???
 *     Hex:     180-188, ???
 */

constexpr uint16_t kHPBase        = 0x180;       // CAN id for high power BMSHP
constexpr uint64_t kHPHcuMaxtBase = 0x186240F3;  // HP Temp (BMS_HCU_MAXT)message ID
constexpr uint64_t kHPHcuInfoBase = 0x186040F3;  // HP BMS_HCU_INFO message ID
constexpr uint64_t kHPHcuMaxvBase = 0x186140F3;  // HP BMS_HCU_MAXV message ID
constexpr uint16_t kHPCellBase    = 0x50F3;      // HP BMS Cell voltage message ending
// 0x180050F3 - 0x184F50F3 [Cell voltage]

struct Data {
  static constexpr uint8_t kTemperatureOffset = 40;
  static constexpr uint8_t kCellNum           = 7;
  uint16_t voltage[kCellNum];
  int8_t temperature;
};

}  // namespace bms

class BMS : public Thread, public CanProccesor, public BMSInterface {
  friend Can;

 public:
  /**
   * @brief Construct a new BMS object
   * @param id  - should correspond to the id sessing on the actual BMS unit
   * @param log - for printing nice messages
   */
  BMS(uint8_t id, Logger &log = utils::System::getLogger());

  ~BMS();

  /**
   * @brief Implement virtual run() from Thread
   * This is used to periodically send request CAN messages
   */
  void run() override;

  // From BMSInterface
  bool isOnline() override;
  void getData(BatteryData *battery) override;

  // From CanProcessor interface
  bool hasId(uint32_t id, bool extended) override;

 private:
  /**
   * @brief Send request CAN message to update data periodically
   */
  void request();

  /**
   * @brief To be called by CAN receive side. BMS processes received CAN
   * message and updates its local data
   *
   * @param message received CAN message to be processed
   */
  void processNewData(utils::io::can::Frame &message) override;

 private:
  bms::Data data_;
  uint8_t id_;                 // my BMS id in (0,..,15)
  uint32_t id_base_;           // my starting CAN id
  uint64_t last_update_time_;  // stores arrival time of CAN response

  // UART CRC checksum method and table
  uint16_t CRC16(const uint8_t* data, uint16_t length) override();
  const static uint16_t kcrcTable[256]={
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
  };

  // for request thread
  Can &can_;
  bool running_;

  // for making sure only one object per BMS unit exist
  static std::vector<uint8_t> existing_ids_;
  static int16_t current_;
  NO_COPY_ASSIGN(BMS);
};

class BMSHP : public CanProccesor, public BMSInterface {
  friend Can;

 public:
  /**
   * @brief Construct a new BMSHP object
   * @param id  - should directly correspond to the CAN id to be used
   * @param log - for printing nice messages
   */
  BMSHP(uint16_t id, Logger &log = utils::System::getLogger());

  // from BMSInterface
  bool isOnline() override;
  void getData(BatteryData *battery) override;

  // from CanProcessor
  bool hasId(uint32_t id, bool extended) override;

 private:
  void processNewData(utils::io::can::Frame &message) override;

 private:
  Logger &log_;
  uint64_t hcu_info_id_;       // HP BMS_HCU_INFO
  uint64_t hcu_max_id_;        // HP BMS_HCU_MAXV
  uint64_t hcu_maxt_id_;       // thermistor expansion module CAN id
  uint16_t cell_id_;           // broadcast message ID
  BatteryData local_data_;     // stores values from CAN
  uint64_t last_update_time_;  // stores arrival time of CAN message
  // for making sure only one object per BMS unit exist
  static std::vector<uint16_t> existing_ids_;
  NO_COPY_ASSIGN(BMSHP);
};

}  // namespace sensors
}  // namespace hyped

#endif  // SENSORS_BMS_HPP_
