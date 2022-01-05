#include "ActsExamples/TrackFitting/TrkrClusterFittingAlgorithm.hpp"

#include <iostream>
#include <map>
#include <random>
#include <stdexcept>

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
#include "boost/program_options.hpp"

#include "ActsExamples/MagneticField/ScalableBField.hpp"


/**
 * Struct that calls the fitting algorithm to get the result of the fit
 */
namespace {

using Updater = Acts::GainMatrixUpdater;
using Smoother = Acts::GainMatrixSmoother;
using Stepper = Acts::EigenStepper<>;
using Propagator = Acts::Propagator<Stepper, Acts::Navigator>;
using Fitter = Acts::KalmanFitter<Propagator>;
using DirectPropagator = Acts::Propagator<Stepper, Acts::DirectNavigator>;
using DirectFitter = Acts::KalmanFitter<DirectPropagator>;

struct TrkrFitterFunctionImpl
    : public ActsExamples::TrkrClusterFittingAlgorithm::TrackFitterFunction {
  Fitter fitter;

  TrkrFitterFunctionImpl(Fitter&& f) : fitter(std::move(f)) {}

  ActsExamples::TrkrClusterFittingAlgorithm::FitterResult operator()(
      const std::vector<std::reference_wrapper<
          const ActsExamples::TrkrClusterSourceLink>>& sourceLinks,
      const ActsExamples::TrackParameters& initialParameters,
      const ActsExamples::TrkrClusterFittingAlgorithm::TrackFitterOptions& options)
      const  {
    return fitter.fit(sourceLinks.begin(), sourceLinks.end(),
		      initialParameters, options);
  };
};

struct DirectedTrkrFitterFunctionImpl
  : public ActsExamples::TrkrClusterFittingAlgorithm::DirectedTrackFitterFunction 
{
  DirectFitter fitter;
  DirectedTrkrFitterFunctionImpl(DirectFitter&& f) : fitter(std::move(f)) {}

  ActsExamples::TrkrClusterFittingAlgorithm::FitterResult operator()(
	  const std::vector<std::reference_wrapper<
	  const ActsExamples::TrkrClusterSourceLink>>& sourceLinks,
	  const ActsExamples::TrackParameters& initialParameters,
	  const ActsExamples::TrkrClusterFittingAlgorithm::TrackFitterOptions& options,
	  const std::vector<const Acts::Surface*>& sSequence) const
  {
    return fitter.fit(sourceLinks.begin(), sourceLinks.end(), 
		      initialParameters, options, sSequence);
  };
};

}  // namespace

/**
 * Function that actually makes the fitting function to be used 
 */

std::shared_ptr<ActsExamples::TrkrClusterFittingAlgorithm::TrackFitterFunction>
ActsExamples::TrkrClusterFittingAlgorithm::makeTrackFitterFunction(
    std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry,
    std::shared_ptr<const Acts::MagneticFieldProvider> magneticField) {
  Stepper stepper(std::move(magneticField));
  Acts::Navigator::Config cfg{trackingGeometry};
  cfg.resolvePassive = false;
  cfg.resolveMaterial = true;
  cfg.resolveSensitive = true;
  Acts::Navigator navigator(cfg);
  Propagator propagator(std::move(stepper), std::move(navigator));
  Fitter trackFitter(std::move(propagator));

  // build the fitter functions. owns the fitter object.
  return std::make_shared<TrkrFitterFunctionImpl >(std::move(trackFitter));
}

std::shared_ptr<
    ActsExamples::TrkrClusterFittingAlgorithm::DirectedTrackFitterFunction>
ActsExamples::TrkrClusterFittingAlgorithm::makeTrackFitterFunction(
    std::shared_ptr<const Acts::MagneticFieldProvider> magneticField) {
  // construct all components for the fitter
  Stepper stepper(std::move(magneticField));
  Acts::DirectNavigator navigator;
  DirectPropagator propagator(std::move(stepper), std::move(navigator));
  DirectFitter fitter(std::move(propagator));

  // build the fitter functions. owns the fitter object.
  return std::make_shared<DirectedTrkrFitterFunctionImpl>(std::move(fitter));
}
