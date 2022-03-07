#include "ActsExamples/Fitting/TrkrClusterOutlierFittingAlgorithm.hpp"

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

  ActsExamples::TrkrClusterOutlierFittingAlgorithm::FitterResult operator()(
       const std::vector<ActsExamples::TrkrClusterSourceLink>& sourceLinks,
       const ActsExamples::TrackParameters& initialParameters,
       const Acts::KalmanFitterOptions<ResidualOutlierFinder>& options) const
  {
    return fitter.fit(sourceLinks, initialParameters, options);
  };
};

template <typename Fitter>
struct DirectedTrkrFitterFunctionImpl
{
  Fitter fitter;

  DirectedTrkrFitterFunctionImpl(Fitter&& f) : fitter(std::move(f)) {}

  ActsExamples::TrkrClusterOutlierFittingAlgorithm::FitterResult operator()(
       const std::vector<ActsExamples::TrkrClusterSourceLink>& sourceLinks,
       const ActsExamples::TrackParameters& initialParameters,
       const Acts::KalmanFitterOptions<ResidualOutlierFinder>& options,
       const std::vector<const Acts::Surface*>& sSequence) const
  {
    return fitter.fit(sourceLinks, initialParameters, options, sSequence);
  };
};

}  // namespace

/**
 * Function that actually makes the fitting function to be used 
 */
ActsExamples::TrkrClusterOutlierFittingAlgorithm::FitterFunction
ActsExamples::TrkrClusterOutlierFittingAlgorithm::makeFitterFunction(
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
        using Fitter             = Acts::KalmanFitter<Propagator, Updater, Smoother, ResidualOutlierFinder>;

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


ActsExamples::TrkrClusterOutlierFittingAlgorithm::DirectedFitterFunction
ActsExamples::TrkrClusterOutlierFittingAlgorithm::makeFitterFunction(
    Options::BFieldVariant magneticField)
{
  using Updater  = Acts::GainMatrixUpdater;
  using Smoother = Acts::GainMatrixSmoother;

  /// Return a new instance of the fitter
  return std::visit(
      [](auto&& inputField) -> DirectedFitterFunction {
	/// Construct some aliases for the components below
        using InputMagneticField = typename std::decay_t<decltype(inputField)>::element_type;
        using MagneticField      = Acts::SharedBField<InputMagneticField>;
        using Stepper            = Acts::EigenStepper<MagneticField>;
        using Navigator          = Acts::DirectNavigator;
        using Propagator         = Acts::Propagator<Stepper, Navigator>;
        using Fitter             = Acts::KalmanFitter<Propagator, Updater, Smoother,ResidualOutlierFinder>;

        /// Make the components for the fitter
        MagneticField field(std::move(inputField));
        Stepper       stepper(std::move(field));
        Navigator     navigator;
        Propagator propagator(std::move(stepper), std::move(navigator));
        Fitter     fitter(std::move(propagator));
			 
        /// Build the fitter function
        return DirectedTrkrFitterFunctionImpl<Fitter>(std::move(fitter));
      },
      std::move(magneticField));
}
