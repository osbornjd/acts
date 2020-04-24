#pragma once

#include <iostream>
#include <map>
#include <random>
#include <stdexcept>
#include <boost/program_options.hpp>

#include <Acts/Fitter/GainMatrixSmoother.hpp>
#include <Acts/Fitter/GainMatrixUpdater.hpp>
#include <Acts/Geometry/GeometryID.hpp>
#include <Acts/MagneticField/ConstantBField.hpp>
#include <Acts/MagneticField/InterpolatedBFieldMap.hpp>
#include <Acts/MagneticField/SharedBField.hpp>
#include <Acts/Propagator/EigenStepper.hpp>
#include <Acts/Propagator/Navigator.hpp>
#include <Acts/Propagator/Propagator.hpp>
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/Utilities/Helpers.hpp>
#include <Acts/Utilities/ParameterDefinitions.hpp>
#include <Acts/Fitter/KalmanFitter.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>

#include "ACTFW/Plugins/BField/ScalableBField.hpp"
#include "ACTFW/EventData/Track.hpp"
#include "ACTFW/Framework/BareAlgorithm.hpp"
#include "ACTFW/Plugins/BField/BFieldOptions.hpp"
#include "ACTFW/EventData/TrkrClusterSourceLink.hpp"

namespace FW {

/**
 * This class contains the information required to run the Kalman fitter
 * with the TrkrClusterSourceLinks. Based on FW::FittingAlgorithm
 */
class TrkrClusterFittingAlgorithm final : public FW::BareAlgorithm
{
public:
  /// Construct some aliases to be used for the fitting results
  using FitterResult
    = Acts::Result<Acts::KalmanFitterResult<FW::Data::TrkrClusterSourceLink>>;
  using FitterFunction
    = std::function<FitterResult(const std::vector<FW::Data::TrkrClusterSourceLink>&,
                                 const FW::TrackParameters&,
                                 const Acts::KalmanFitterOptions<Acts::VoidOutlierFinder>&)>;

  /// Create fitter function
  static FitterFunction
    makeFitterFunction(
      std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry,
      FW::Options::BFieldVariant                    magneticField,
      Acts::Logging::Level                          lvl);

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
