/*
 * Author:
 * Organisation: HYPED
 * Date: 06/03/2020
 * Description: Outlier detection class
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

#ifndef NAVIGATION_OUTLIER_DETECTION_HPP_
#define NAVIGATION_OUTLIER_DETECTION_HPP_

#include <vector>
#include "data/data.hpp"
#include "navigation/navigation.hpp"

namespace hyped {
using data::Data;
using data::DataPoint;
using data::ImuData;
using data::ModuleStatus;
using data::NavigationType;
using data::NavigationVector;
using utils::Logger;
using std::array;
using std::vector;

namespace navigation {

  class OutlierDetection {
    public:
      typedef vector<NavigationType> OutlierVector;
      OutlierDetection();
      // Constants used in modified Z-socre algorithm for outlier detection
      static constexpr float kMeanADCoeficient = 1.253314;
      static constexpr float kMedianADCoeficient = 1.486;
      int deadIMUs();
      bool criticalFailure();
      void detectOutliers();   // it gets m_zscore
      NavigationType getMedianAdjusted();
      NavigationType getMean();
      NavigationType getMedianAD();
      NavigationType getMeanAD(NavigationType mean);
      Navigation::NavigationArray& dataArray_;
      Navigation::NavigationArray getDataArrayCopy();

    private:
      OutlierVector values_;
      // Array of booleans to signify which IMUs are reliable or faulty
      array<bool, Sensors::kNumImus> isSensorDead_;           // same as sensor_reliable
      vector<uint32_t> sensorOutlierCounter;
      NavigationType getMedian(Navigation::NavigationArray dataArr);
      // WIP
  };
}
}

#endif  // NAVIGATION_OUTLIER_DETECTION_HPP_
