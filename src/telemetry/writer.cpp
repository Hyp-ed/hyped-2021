/*
 * Author: Robertas Norkus
 * Organisation: HYPED
 * Date: January 2020
 * Description:
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

#include <string>

#include "writer.hpp"

namespace hyped {
namespace telemetry {

  // additional data points that are displayed in the GUI data section
  // FEEL FREE TO EDIT. More info: https://github.com/Hyp-ed/hyped-2020/wiki/State-Machine
  void Writer::packAdditionalData()
  {
    rjwriter_.Key("additional_data");
    rjwriter_.StartArray();

    // edit below


    // edit above

    rjwriter_.EndArray();
  }

  // crucial data points that are displayed in various fixed GUI points
  // NOT EDITABLE
  void Writer::packCrucialData()
  {
    rjwriter_.Key("crucial_data");
    rjwriter_.StartArray();

    add("distance", 0, 1250, "m", 0);
    add("velocity", 0, 100, "m/s", 0);
    add("acceleration", -20, 20, "m/s^2", 0);
    add("status", true);

    rjwriter_.EndArray();
  }

  // status data points that are displayed in the GUI status tab
  // NOT EDITABLE
  void Writer::packStatusData()
  {
    rjwriter_.Key("status_data");
    rjwriter_.StartArray();

    rjwriter_.EndArray();
  }

  Writer::Writer()
  {
    rapidjson::StringBuffer sb_;
    rapidjson::Writer<rapidjson::StringBuffer> rjwriter_(sb_);
  }

  void Writer::start()
  {
    rjwriter_.StartObject();
  }

  void Writer::end()
  {
    rjwriter_.EndObject();
  }

  std::string Writer::getString()
  {
    return sb_.GetString();
  }

  void Writer::add(std::string name, int min, int max, std::string units, int value)
  {
    rjwriter_.StartObject();
    rjwriter_.Key("name");
    rjwriter_.String(name.c_str());
    rjwriter_.Key("min");
    rjwriter_.Int(min);
    rjwriter_.Key("max");
    rjwriter_.Int(max);
    rjwriter_.Key("units");
    rjwriter_.String(units.c_str());
    rjwriter_.Key("value");
    rjwriter_.Int(value);
    rjwriter_.EndObject();
  }

  void Writer::add(std::string name, float min, float max, std::string units, float value)
  {
    rjwriter_.StartObject();
    rjwriter_.Key("name");
    rjwriter_.String(name.c_str());
    rjwriter_.Key("min");
    rjwriter_.Double(min);
    rjwriter_.Key("max");
    rjwriter_.Double(max);
    rjwriter_.Key("units");
    rjwriter_.String(units.c_str());
    rjwriter_.Key("value");
    rjwriter_.Double(value);
    rjwriter_.EndObject();
  }

  void Writer::add(std::string name, bool value)
  {
    rjwriter_.StartObject();
    rjwriter_.Key("name");
    rjwriter_.String(name.c_str());
    rjwriter_.Key("value");
    rjwriter_.Bool(value);
    rjwriter_.EndObject();
  }

  void Writer::add(std::string name, data::State value)
  {
    rjwriter_.StartObject();
    rjwriter_.Key("name");
    rjwriter_.String(name.c_str());
    rjwriter_.Key("value");
    rjwriter_.String(convertStateMachineState(value).c_str());
    rjwriter_.EndObject();
  }

  void Writer::add(std::string name, data::ModuleStatus value)
  {
    rjwriter_.StartObject();
    rjwriter_.Key("name");
    rjwriter_.String(name.c_str());
    rjwriter_.Key("value");
    rjwriter_.String(convertModuleStatus(value).c_str());
    rjwriter_.EndObject();
  }

  std::string Writer::convertStateMachineState(data::State state)
  {
    switch (state) {
    case data::State::kInvalid:
      return "INVALID";
      break;
    case data::State::kEmergencyBraking:
      return "EMERGENCY_BRAKING";
      break;
    case data::State::kFailureStopped:
      return "FAILURE_STOPPED";
      break;
    case data::State::kIdle:
      return "IDLE";
      break;
    case data::State::kCalibrating:
      return "CALIBRATING";
      break;
    case data::State::kRunComplete:
      return "RUN_COMPLETE";
      break;
    case data::State::kFinished:
      return "FINISHED";
      break;
    case data::State::kReady:
      return "READY";
      break;
    case data::State::kAccelerating:
      return "ACCELERATING";
      break;
    case data::State::kNominalBraking:
      return "NOMINAL_BRAKING";
      break;
    case data::State::kExiting:
      return "EXITING";
      break;
    default:
      return "";
      break;
    }
  }

  std::string Writer::convertModuleStatus(data::ModuleStatus module_status)
  {
    switch (module_status) {
    case data::ModuleStatus::kStart:
      return "START";
      break;
    case data::ModuleStatus::kInit:
      return "INIT";
      break;
    case data::ModuleStatus::kReady:
      return "READY";
      break;
    case data::ModuleStatus::kCriticalFailure:
      return "CRITICAL_FAILURE";
      break;
    default:
      return "";
      break;
    }
  }

}  // namespace telemetry
}  // namespace hyped
