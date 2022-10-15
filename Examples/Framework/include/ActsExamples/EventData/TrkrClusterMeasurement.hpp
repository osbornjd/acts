// This file is part of the Acts project.
//
// Copyright (C) 2020 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "Acts/EventData/Measurement.hpp"
#include "Acts/EventData/MultiTrajectory.hpp"
#include "Acts/EventData/SourceLink.hpp"

#include "ActsExamples/EventData/TrkrClusterSourceLink.hpp"
#include "ActsExamples/EventData/Measurement.hpp"

#include <cassert>
#include <vector>

namespace ActsExamples {

/// Calibrator to convert an index source link to a measurement.
class TrkrClusterMeasurementCalibrator {
 public:
  /// Construct an invalid calibrator. Required to allow copying.
  TrkrClusterMeasurementCalibrator() = default;
  /// Construct using a user-provided container to chose measurements from.
  TrkrClusterMeasurementCalibrator(const MeasurementContainer& measurements)
      : m_measurements(&measurements) {}

  /// Find the measurement corresponding to the source link.
  ///
  /// @tparam parameters_t Track parameters type
  /// @param gctx The geometry context (unused)
  /// @param trackState The track state to calibrate
  void calibrate(const Acts::GeometryContext& /*gctx*/,
                 Acts::MultiTrajectory::TrackStateProxy trackState) const {
    std::cout << "should have checked here"<<std::endl;
    const auto& sourceLink =
        static_cast<const TrkrClusterSourceLink&>(trackState.uncalibrated());
    std::cout << "is this ever called"<<std::endl;
    assert(m_measurements and
           "Undefined measurement container in DigitizedCalibrator");
    std::cout << "undefined measurement container"<<std::endl;
    assert((sourceLink.index() < m_measurements->size()) and
           "Source link index is outside the container bounds");
    std::cout << "Joe: accessing source link " << sourceLink.index()
	      << std::endl;
    std::visit(
        [&trackState](const auto& meas) { trackState.setCalibrated(meas);
	  std::cout << "Meas params " << meas.parameters().transpose() << std::endl;},
        (*m_measurements)[sourceLink.index()]);
  }

 private:
  // use pointer so the calibrator is copyable and default constructible.
  const MeasurementContainer* m_measurements = nullptr;
};

}  // namespace ActsExamples
