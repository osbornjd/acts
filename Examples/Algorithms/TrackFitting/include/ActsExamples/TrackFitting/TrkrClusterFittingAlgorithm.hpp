#pragma once

#include <iostream>
#include <map>
#include <random>
#include <stdexcept>
#include <boost/program_options.hpp>

#include "Acts/Definitions/TrackParametrization.hpp"
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
#include "Acts/TrackFitting/KalmanFitter.hpp"
#include "Acts/Geometry/TrackingGeometry.hpp"

#include "ActsExamples/MagneticField/ScalableBField.hpp"
#include "ActsExamples/EventData/Track.hpp"
#include "ActsExamples/Framework/BareAlgorithm.hpp"
#include "ActsExamples/MagneticField/MagneticFieldOptions.hpp"
#include "ActsExamples/EventData/TrkrClusterSourceLink.hpp"

namespace ActsExamples {

/**
 * This class contains the information required to run the Kalman fitter
 * with the TrkrClusterSourceLinks. Based on ActsExamples::FittingAlgorithm
 */
class TrkrClusterFittingAlgorithm final : public BareAlgorithm
{
 public:
  /// Construct some aliases to be used for the fitting results
  using TrackFitterOptions = Acts::KalmanFitterOptions;
  using FitterResult
    = Acts::Result<Acts::KalmanFitterResult>;
  
  class TrackFitterFunction {
    public:
      virtual ~TrackFitterFunction() = default;
      virtual FitterResult operator()(
         const std::vector<std::reference_wrapper<const TrkrClusterSourceLink>>&,
       	 const TrackParameters&, const TrackFitterOptions&) const = 0;
  };

  class DirectedTrackFitterFunction {
    public:
      virtual ~DirectedTrackFitterFunction() = default;
      virtual FitterResult operator()(
          const std::vector<std::reference_wrapper<const TrkrClusterSourceLink>>&,
          const TrackParameters&, const TrackFitterOptions&,
          const std::vector<const Acts::Surface*>&) const = 0;
  };

  static std::shared_ptr<TrackFitterFunction> makeTrackFitterFunction(
      std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry,
      std::shared_ptr<const Acts::MagneticFieldProvider> magneticField);

  static std::shared_ptr<DirectedTrackFitterFunction> makeTrackFitterFunction(
      std::shared_ptr<const Acts::MagneticFieldProvider> magneticField);


  struct Config {
    bool directNavigation;
    std::shared_ptr<TrackFitterFunction> fit;
    std::shared_ptr<DirectedTrackFitterFunction> dfit;
    std::shared_ptr<const Acts::TrackingGeometry> tGeometry;
    bool multipleScattering = true;
    bool energyLoss = true;
  };
  /// Constructor 
  TrkrClusterFittingAlgorithm(Config cfg, Acts::Logging::Level lvl);


 private:
  Config m_cfg;

  /// Helper function to call correct FitterFunction
  FitterResult fitTrack(
      const std::vector<std::reference_wrapper<
          const ActsExamples::TrkrClusterSourceLink>>& sourceLinks,
      const ActsExamples::TrackParameters& initialParameters,
      const TrackFitterOptions& options,
      const std::vector<const Acts::Surface*>& surfSequence) const;

};

inline ActsExamples::TrkrClusterFittingAlgorithm::FitterResult
ActsExamples::TrkrClusterFittingAlgorithm::fitTrack(
    const std::vector<std::reference_wrapper<
        const ActsExamples::TrkrClusterSourceLink>>& sourceLinks,
    const ActsExamples::TrackParameters& initialParameters,
    const Acts::KalmanFitterOptions& options,
    const std::vector<const Acts::Surface*>& surfSequence) const {
  if (m_cfg.directNavigation) {
    return (*m_cfg.dfit)(sourceLinks, initialParameters, options, surfSequence);
  }

  return (*m_cfg.fit)(sourceLinks, initialParameters, options);
}

}
