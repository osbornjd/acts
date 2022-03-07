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

struct ResidualOutlierFinder {
  std::vector<std::pair<long unsigned int, double>> chi2Cuts;
  int verbosity = 0;

  template <typename track_state_t>
  bool operator()(const track_state_t& state) const {
    
    /// can't determine an outlier without a measusrement or prediction from KF
    if(not state.hasCalibrated() or not state.hasPredicted())
      return false;

    /// residuals between measurement and predicted state, local coord
    // residuals = state.calibrated() - state.projector() * state.predicted();
    auto distance = (state.calibrated() - state.projector() * state.predicted()).norm();

    /// Get the relevant state parameters
    const auto& projector = state.projector();
    const auto& predictedCov = state.predictedCovariance();
    const auto& stateCov = state.effectiveCalibratedCovariance();
    
    /// Calculate the chi2 between the prediction and measurement, similar to
    /// Acts::MeasurementSelector.operator()
    double chi2 = ((state.calibrated() - state.projector() * state.predicted()).transpose() * 
		   (( stateCov + projector * predictedCov * projector.transpose())).inverse() * (state.calibrated() - state.projector() * state.predicted()))
                   .eval()(0,0);

    if(verbosity > 2)
      {
	std::cout << "geoid : " << state.referenceSurface().geometryId() << std::endl;
	std::cout << "Distance between prediction and measurement is: " << distance << std::endl;
	std::cout << "chi2 " << chi2  << std::endl;
      }

    auto volID = state.referenceSurface().geometryId().volume();

    for(auto& pair : chi2Cuts)
      {
	if(verbosity > 2)
	  {
	    std::cout << "VolID : " << volID << " checking against " << pair.first 
		      << ",   chicut " << pair.second << " with chi2 " << chi2 << std::endl;
	  }

	if(volID == pair.first)
	  {
	    return chi2 >= pair.second;
	  }
      }
    
    /// If it's some other (unknown) detector default to keeping the measurement
    return false;
  }
};


namespace ActsExamples {

/**
 * This class contains the information required to run the Kalman fitter
 * with the TrkrClusterSourceLinks. Based on ActsExamples::FittingAlgorithm
 */
class TrkrClusterOutlierFittingAlgorithm : public BareAlgorithm
{
 public:
  /// Construct some aliases to be used for the fitting results
  using FitterResult
    = Acts::Result<Acts::KalmanFitterResult<ActsExamples::TrkrClusterSourceLink>>;
  using FitterFunction
    = std::function<FitterResult(
		    const std::vector<ActsExamples::TrkrClusterSourceLink>&,
		    const TrackParameters&,
		    const Acts::KalmanFitterOptions<ResidualOutlierFinder>&)>;

  using DirectedFitterFunction
    = std::function<FitterResult(
		    const std::vector<ActsExamples::TrkrClusterSourceLink>&,
		    const TrackParameters&,
		    const Acts::KalmanFitterOptions<ResidualOutlierFinder>&,
		    const std::vector<const Acts::Surface*>&)>;
      

  /// Create fitter function
  static FitterFunction makeFitterFunction(
      std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry,
      Options::BFieldVariant magneticField);

  static DirectedFitterFunction makeFitterFunction(
      Options::BFieldVariant magneticField);

  struct Config
  {
    FitterFunction fit;
    DirectedFitterFunction dFit;
  };

  /// Constructor 
  TrkrClusterOutlierFittingAlgorithm(Config cfg, Acts::Logging::Level lvl);


private:
  Config m_cfg;
};

}
