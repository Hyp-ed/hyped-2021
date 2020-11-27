/*
 * Author: Karthik Narayanan
 * Organisation: HYPED
 * Date: 30.10.2020
 * Description: Handles the communication with the CAN Bus
 *
 *    Copyright 2020 HYPED
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

#include "propulsion/can/can_sender.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"

using hyped::utils::Logger;
using hyped::motor_control::CanSender;
using hyped::utils::io::can::Frame;


bool sendTestMessage(CanSender& sender, Logger& log_)
{
  Frame message;
  log_.INFO("MOTOR", "Sending Message");
  message.id = 1536;
  message.len = 5;
  message.extended = false;
  // const char testStr = "Hello";
  message.data[0] = 'H';
  message.data[1] = 'e';
  message.data[2] = 'l';
  message.data[3] = 'l';
  message.data[4] = 'o';
  return sender.sendMessage(message);
}

int main(int argc,char* argv[]) {
  hyped::utils::System::parseArgs(argc, argv);
  Logger& log_ = hyped::utils::System::getLogger();
  CanSender sender(log_,0);
  sender.registerController();
  sendTestMessage(sender, log_);

  return 1;
}