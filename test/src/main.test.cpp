/*
 * Author: QA team
 * Organisation: HYPED
 * Date:
 * Description:
 *
 *    Copyright 2019 HYPED
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

#include <iostream>

#include "gtest/gtest.h"
#include "utils/system.hpp"

int main(int argc, char **argv)
{
  // Initialising single system instance to be used across all tests
  // construct argument list to pass to System::parseArgs
  static char arg0[] = "testing.cpp";
  static char arg1[] = "--config=test/config.txt";
  char *args[]       = {arg0, arg1};
  hyped::utils::System::parseArgs(2, args);

  std::cout << "running\n";
  ::testing::InitGoogleTest(&argc, argv);

  return (RUN_ALL_TESTS());
}
