/*
 * Author: Gregory Dayao, Yeyao Liu
 * Organisation: HYPED
 * Date:
 * Description: Main file for Magnetometer
 *
 *    Copyright 2019 HYPED
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
#include <algorithm>
#include <vector>

#include "sensors/magnetometer.hpp"
#include "utils/concurrent/thread.hpp"
#include "utils/math/statistics.hpp"
#include "utils/interface_factory.hpp"

// user bank addresse
constexpr uint8_t kRegBankSel               = 0x7F;

// Accelerometer addresses
constexpr uint8_t kAccelXoutH               = 0x2D;   // userbank 0

constexpr uint8_t kAccelConfig              = 0x14;   // userbank 2
constexpr uint8_t kAccelScale               = 0x02;   // +/- 4g
constexpr uint8_t kAccelSmplrtDiv1          = 0x10;   // userbank 2
constexpr uint8_t kAccelSmplrtDiv2          = 0x11;   // userbank 2

constexpr uint8_t kWhoAmIMagnetometer                = 0x00;   // sensor to be at this address, userbank 0
// data to be at these addresses when read from sensor else not initialised
constexpr uint8_t kWhoAmIResetValue         = 0xEA;   // userbank 0

// Power Management
constexpr uint8_t kPwrMgmt1                 = 0x06;   // userbank 0
constexpr uint8_t kPwrMgmt2                 = 0x07;   // userbank 0

// Configuration
constexpr uint8_t kReadFlag                 = 0x80;   // unable to find in datasheet

// Configuration bits Magnetometer
// constexpr uint8_t kBitsFs250Dps             = 0x00;
// constexpr uint8_t kBitsFs500Dps             = 0x08;
// constexpr uint8_t kBitsFs1000Dps            = 0x10;
// constexpr uint8_t kBitsFs2000Dps            = 0x18;
constexpr uint8_t kBitsFs2G                 = 0x00;   // for accel_config
constexpr uint8_t kBitsFs4G                 = 0x02;
constexpr uint8_t kBitsFs8G                 = 0x04;
constexpr uint8_t kBitsFs16G                = 0x06;

// Resets the device to defaults
constexpr uint8_t kBitHReset                = 0x80;    // for pwr_mgmt


// values for FIFO
// constexpr uint8_t kFifoEnable1              = 0x66;   // userbank 0
constexpr uint8_t kFifoEnable2              = 0x67;   // userbank 0
constexpr uint8_t kFifoReset                = 0x68;   // userbank 0
constexpr uint8_t kFifoMode                 = 0x69;   // userbank 0
constexpr uint8_t kFifoCountH               = 0x70;   // userbank 0
constexpr uint8_t kFifoRW                   = 0x72;   // userbank 0
constexpr uint8_t kDataRdyStatus            = 0x74;   // userbank 0
constexpr uint8_t kUserCtrl                 = 0x03;   // userbank 0
// constexpr uint8_t kIntEnable2 = 0x12;    // userbank 0, for FIFO overflow, read = 0x10


// Magnetometer stuff
constexpr uint8_t kI2cMstCtrl               = 0x01;   // userbank 3
constexpr uint8_t kI2cMstDelayCtrl          = 0x07;   // userbank 3
constexpr uint8_t kI2cSlv0Addr              = 0x03;   // userbank 3
constexpr uint8_t kI2cSlv0Reg               = 0x04;   // userbank 3
constexpr uint8_t kI2cSlv0Do                = 0x06;   // userbank 3
constexpr uint8_t kI2cSlv0Ctrl              = 0x05;   // userbank 3
constexpr uint8_t kFifoEnable1              = 0x66;   // userbank 0
constexpr uint8_t kExtSlvSensData00         = 0x3B;   // userbank 0
// constexpr uint8_t kExtSlvSensData00         = 0x3B;   // userbank 0
constexpr uint8_t kExtSlvSensData06         = 0x41;   // userbank 0



namespace hyped {

// utils::io::gpio::Direction kDirection = utils::io::gpio::kOut;
using utils::concurrent::Thread;
using utils::math::OnlineStatistics;
using data::NavigationVector;

namespace sensors {

Magnetometer::Magnetometer(Logger& log, uint32_t pin, bool is_fifo)
    : spi_(SPI::getInstance()),
    log_(log),
    gpio_(pin, utils::io::gpio::kOut, log),
    pin_(pin),
    is_fifo_(is_fifo),
    is_online_(false)
{
  log_.DBG1("Magnetometer pin: ", "%d", pin);
  log_.INFO("Magnetometer", "Creating Magnetometer sensor now:");
  init();
}

// sources I used, https://devzone.nordicsemi.com/f/nordic-q-a/36615/invensense-icm-20948
// https://github.com/kriswiner/MPU9250/issues/367

void Magnetometer::init()
{
  // Set pin high
  gpio_.set();

  selectBank(0);

  writeByte(kPwrMgmt1, kBitHReset);   // Reset Device
  Thread::sleep(200);
  // Test connection
  bool check_init = whoAmI();

  writeByte(kPwrMgmt1, 0x01);         // autoselect clock source
  // writeByte(kIntPinConfig, 0xC0); // int pin config

  writeByte(kPwrMgmt2, 0x07);         // enable acc, disable gyro

  // Digital Motion Processor disabled to enable FIFO
  // writeByte(kUserCtrl, 0x08);         // reset DMP
  // writeByte(kUserCtrl, 0x80);         // enable DMP

  // acceleration configurations
  selectBank(2);

  writeByte(kAccelConfig, 0x01);    // reset val

  // DLPF
  // writeByte(kAccelConfig, 0x09);    // LPF and DLPF configuration
  writeByte(kAccelConfig, 0x08);       // reference low pass filter config table

  selectBank(0);
  // Step 1.

  // Enable the I2C master
  writeByte(kUserCtrl, 0x20);
  Thread::sleep(10);
  selectBank(3);

  // writeByte(kI2cMstCtrl, 0x97); // Enable the I2C Multi Master
  writeByte(kI2cMstCtrl, 0x07); // Enable the I2C Multi Master
  Thread::sleep(100);
  writeByte(kI2cSlv0Addr, 0x0C); // Set the slave 0 addr of mag
  Thread::sleep(100);
  writeByte(kI2cSlv0Reg, 0x32); // AK09916 control 3
  Thread::sleep(100);
  writeByte(kI2cSlv0Do, 0x01); // reset the mag
  Thread::sleep(100);
  writeByte(kI2cSlv0Ctrl, 0x81); // enable i2c, set 1 byte
  Thread::sleep(100);
  writeByte(kI2cSlv0Reg, 0x31); // AK09916 control 2
  Thread::sleep(100);
  writeByte(kI2cSlv0Do, 0x08); // set the value to continous measurement
  Thread::sleep(100);
  writeByte(kI2cSlv0Ctrl, 0x81); // enable i2c, set 1 byte
  Thread::sleep(100);


  setAcclScale();

  if (check_init) {
    log_.ERR("Magnetometer", "Magnetometer sensor %d created. Initialisation complete.", pin_);
    selectBank(0);
  } else {
    log_.ERR("Magnetometer", "ERROR: Magnetometer sensor %d not initialised.", pin_);
  }
}

bool Magnetometer::whoAmI()
{
  uint8_t data;
  int send_counter;

  for (send_counter = 1; send_counter < 10; send_counter++) {
    readByte(kWhoAmIMagnetometer, &data);
    log_.DBG1("Magnetometer", "Magnetometer connected to SPI, data: %d", data);
    if (data == kWhoAmIResetValue) {
      is_online_ = true;
      break;
    } else {
      log_.DBG1("Magnetometer", "Cannot initialise. Who am I is incorrect");
      is_online_ = false;
      Thread::yield();
    }
  }

  if (!is_online_) {
    log_.ERR("Magnetometer", "Cannot initialise who am I. Sensor %d offline", pin_);
  }
  return is_online_;
}

Magnetometer::~Magnetometer()
{
  log_.INFO("Magnetometer", "Deconstructing sensor %d object", pin_);
}

void Magnetometer::selectBank(uint8_t switch_bank)
{
  writeByte(kRegBankSel, (switch_bank << 4));
  // log_.DBG1("Magnetometer", "bank switch = %d", (switch_bank << 4));
  user_bank_ = switch_bank;
  log_.DBG1("Magnetometer", "User bank switched to %u", user_bank_);
}

void Magnetometer::writeByte(uint8_t write_reg, uint8_t write_data)
{
  // ',' instead of ';' is to inform the compiler not to reorder function calls
  // chip selects signals must have exact ordering with respect to the spi access
  select(),
  spi_.write(write_reg, &write_data, 1),
  deSelect();
}

void Magnetometer::readByte(uint8_t read_reg, uint8_t *read_data)
{
  select(),
  spi_.read(read_reg | kReadFlag, read_data, 1),
  deSelect();
}

void Magnetometer::readBytes(uint8_t read_reg, uint8_t *read_data, uint8_t length)
{
  select(),
  spi_.read(read_reg | kReadFlag, read_data, length),
  deSelect();
}

void Magnetometer::select()
{
  gpio_.clear();
}
void  Magnetometer::deSelect()
{
  gpio_.set();
}

void Magnetometer::setAcclScale()
{
  // userbank 2
  selectBank(2);
  uint8_t data;
  readByte(kAccelConfig, &data);
  writeByte(kAccelConfig, data | kAccelScale);
  // set accel sample rate divider to maximise sample rate (1125 Hz)
  writeByte(kAccelSmplrtDiv1, 0x00);
  writeByte(kAccelSmplrtDiv2, 0x00);

  switch (kAccelScale) {
    case kBitsFs2G:
      acc_divider_ = 16384;
    break;
    case kBitsFs4G:
      acc_divider_ = 8192;
    break;
    case kBitsFs8G:
      acc_divider_ = 4096;
    break;
    case kBitsFs16G:
      acc_divider_ = 2048;
    break;
  }
}

void Magnetometer::getData(ImuData* data)
{
  if (is_online_) {
      log_.DBG2("Magnetometer", "Getting Magnetometer data");
      auto& acc = data->acc;
      uint8_t response[8];
      int16_t bit_data;
      float value;
      int i;
      float accel_data[3];

      selectBank(0);
      readBytes(kAccelXoutH, response, 8);
      for (i = 0; i < 3; i++) {
        bit_data = ((int16_t) response[i*2] << 8) | response[i*2+1];
        value = static_cast<float>(bit_data);
        accel_data[i] = value/acc_divider_  * 9.80665;
      }
      data->operational = is_online_;
      acc[0] = accel_data[0];
      acc[1] = accel_data[1];
      acc[2] = accel_data[2];
  } else {
    // Try and turn the sensor on again
    log_.ERR("Magnetometer", "Sensor not operational, trying to turn on sensor");
    init();
  }
}

void Magnetometer::getMagData(ImuData* data)
{
  if (is_online_) {
    log_.DBG2("Magnetometer", "Getting Magnetometer data");
    uint8_t response[8];
    int16_t bit_data;
    float value;
    int i;
    float accel_data[3];
    // uint8_t random;

    selectBank(3);
    writeByte(kI2cSlv0Addr, 0x8C); // Set for read
    writeByte(kI2cSlv0Reg, 0x11);
    writeByte(kI2cSlv0Ctrl, 0x88);
    Thread::sleep(100);

    selectBank(0);
    readBytes(kExtSlvSensData00, response, 8);
    // readByte(kExtSlvSensData06, &random);

    // for (i = 0; i < 3; i++) {
    //   bit_data = ((int16_t) response[i*2+1] << 8) | response[i*2];
    //   value = static_cast<float>(bit_data);
    //   accel_data[i] = value;
    // }
    accel_data[0] = static_cast<float>(((int16_t) response[1] << 8) | response[0]);
    accel_data[1] = static_cast<float>(((int16_t) response[3] << 8) | response[2]);
    accel_data[2] = static_cast<float>(((int16_t) response[5] << 8) | response[4]);
    data->operational = is_online_;
    data->acc[0] = accel_data[0] * 0.15;
    data->acc[1] = accel_data[1] * 0.15;
    data->acc[2] = accel_data[2] * 0.15;
  } else {
    // Try and turn the sensor on again
    log_.ERR("Magnetometer", "Sensor not operational, trying to turn on sensor");
    init();
  }
}

}}  // namespace hyped::sensors
