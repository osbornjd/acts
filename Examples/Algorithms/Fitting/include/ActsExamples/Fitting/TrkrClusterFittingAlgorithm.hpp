#pragma once

#include <iostream>
#include <map>
#include <random>
#include <stdexcept>
#include <boost/program_options.hpp>

#include "Acts/TrackFitting/GainMatrixSmoother.hpp"
#include "Acts/TrackFitting/GainMatrixUpdater.hpp"
#include "Acts/Geometry/GeometryIdentifier.hpp"
#include "Acts/MagneticField/ConstantBField.hpp"
#include "Acts/MagneticField/InterpolatedBFieldMap.hpp"
#include "Acts/MagneticField/SharedBField.hpp"
#include "Acts/Propagator/EigenStepper.hpp"
#include "Acts/Propagator/Navigator.hpp"
#include "Acts/Propagator/Propagator.hpp"
#include "Acts/Surfaces/Surface.hpp"
#include "Acts/Utilities/Helpers.hpp"
#include "Acts/Utilities/ParameterDefinitions.hpp"
#include "Acts/TrackFitting/KalmanFitter.hpp"
#include "Acts/Geometry/TrackingGeometry.hpp"

#include "ActsExamples/Plugins/BField/ScalableBField.hpp"
#include "ActsExamples/EventData/Track.hpp"
#include "ActsExamples/Framework/BareAlgorithm.hpp"
#include "ActsExamples/Plugins/BField/BFieldOptions.hpp"
#include "ActsExamples/EventData/TrkrClusterSourceLink.hpp"

namespace ActsExamples {

/**
 * This class contains the information required to run the Kalman fitter
 * with the TrkrClusterSourceLinks. Based on ActsExamples::FittingAlgorithm
 */
class TrkrClusterFittingAlgorithm : public BareAlgorithm
{
 public:
  /// Construct some aliases to be used for the fitting results
  using FitterResult
    = Acts::Result<Acts::KalmanFitterResult<ActsExamples::TrkrClusterSourceLink>>;
  using FitterFunction
    = std::function<FitterResult(
		    const std::vector<ActsExamples::TrkrClusterSourceLink>&,
		    const TrackParameters&,
		    const Acts::KalmanFitterOptions<Acts::VoidOutlierFinder>&)>;

  /// Create fitter function
  static FitterFunction makeFitterFunction(
      std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry,
      Options::BFieldVariant magneticField);

  struct Config
  {
    FitterFunction fit;
  };

  /// Constructor 
  TrkrClusterFittingAlgorithm(Config cfg, Acts::Logging::Level lvl);


private:
  Config m_cfg;
};

}
