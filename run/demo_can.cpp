/*
 * Author: Karthik Narayanan
 * Organisation: HYPED
 * Date: 30.10.2020
 * Description: Sends and recieves large amount of CAN messages 
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
#include "propulsion/controller.hpp"

using hyped::utils::Logger;
using hyped::motor_control::CanSender;
using hyped::utils::io::can::Frame;
using hyped::motor_control::Controller;


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

bool recieveTestMessage(CanSender& sender, Logger& log_, uint8_t node_id)
{
  Frame message;
  log_.INFO("MOTOR", "Recieving Message");
  message.id = 0x580 + node_id;
  message.len = 8;
  message.extended = false;
  // const char testStr = "Hello";
  message.data[1] = 0x6c;
  message.data[2] = 0x60;
  message.data[3] = 0x00;
  message.data[4] = 0x01;
  message.data[5] = 0x01;
  message.data[6] = 0x01;
  message.data[7] = 0x01;
  sender.processNewData(message);
  return true;
}

int main(int argc,char* argv[]) {
  hyped::utils::System::parseArgs(argc, argv);
  Logger& log_ = hyped::utils::System::getLogger();
  CanSender sender(log_,0);
  Controller controller(log_, argc);
  sender.registerController();
  // Frame receive_msg();
  for (int i = 0; i <= 100000; i++) {
    sendTestMessage(sender, log_);
    recieveTestMessage(sender, log_, controller.getNode_id());
    log_.INFO("DEMO", "Message received, node id = %d", controller.getNode_id());
  }


  return 1;
}
