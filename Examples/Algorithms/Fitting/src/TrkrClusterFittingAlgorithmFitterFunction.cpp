#include "ActsExamples/Fitting/TrkrClusterFittingAlgorithm.hpp"

#include <iostream>
#include <map>
#include <random>
#include <stdexcept>

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
#include "boost/program_options.hpp"

#include "ActsExamples/Plugins/BField/ScalableBField.hpp"


/**
 * Struct that calls the fitting algorithm to get the result of the fit
 */
namespace {
template <typename Fitter>
struct TrkrFitterFunctionImpl
{
  Fitter fitter;

  TrkrFitterFunctionImpl(Fitter&& f) : fitter(std::move(f)) {}

  ///  Calls Acts standard fitter with navigation on each layer
  ActsExamples::TrkrClusterFittingAlgorithm::FitterResult operator()(
       const std::vector<ActsExamples::TrkrClusterSourceLink>& sourceLinks,
       const ActsExamples::TrackParameters& initialParameters,
       const Acts::KalmanFitterOptions<Acts::VoidOutlierFinder>& options) const
  {
    return fitter.fit(sourceLinks, initialParameters, options);
  };

  /// Calls Acts direct navigation fitter with a sequence of surfaces that
  /// are specified. Navigation will proceed from surface to surface and must
  /// be already sorted/ordered
  ActsExamples::TrkrClusterFittingAlgorithm::FitterResult operator()(
       const std::vector<ActsExamples::TrkrClusterSourceLink>& sourceLinks,
       const ActsExamples::TrackParameters& initialParameters,
       const Acts::KalmanFitterOptions<Acts::VoidOutlierFinder>& options,
       const std::vector<const Acts::Surface*>& surfSequence) const
  {
    return fitter.fit(sourceLinks, initialParameters, options, surfSequence);
  }
};
}  // namespace

/**
 * Function that actually makes the fitting function to be used 
 */
ActsExamples::TrkrClusterFittingAlgorithm::FitterFunction
ActsExamples::TrkrClusterFittingAlgorithm::makeFitterFunction(
    std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry,
    Options::BFieldVariant magneticField)
{
  using Updater  = Acts::GainMatrixUpdater;
  using Smoother = Acts::GainMatrixSmoother;

  /// Return a new instance of the fitter
  return std::visit(
      [trackingGeometry](auto&& inputField) -> FitterFunction {
	/// Construct some aliases for the components below
        using InputMagneticField = typename std::decay_t<decltype(inputField)>::element_type;
        using MagneticField      = Acts::SharedBField<InputMagneticField>;
        using Stepper            = Acts::EigenStepper<MagneticField>;
        using Navigator          = Acts::Navigator;
        using Propagator         = Acts::Propagator<Stepper, Navigator>;
        using Fitter             = Acts::KalmanFitter<Propagator, Updater, Smoother>;

        /// Make the components for the fitter
        MagneticField field(std::move(inputField));
        Stepper       stepper(std::move(field));
        Navigator     navigator(trackingGeometry);
        navigator.resolvePassive   = false;
        navigator.resolveMaterial  = true;
        navigator.resolveSensitive = true;
        Propagator propagator(std::move(stepper), std::move(navigator));
        Fitter     fitter(std::move(propagator));
			 
        /// Build the fitter function
        return TrkrFitterFunctionImpl<Fitter>(std::move(fitter));
      },
      std::move(magneticField));
}
