#include "ACTFW/Fitting/TrkrClusterFittingAlgorithm.hpp"

#include <iostream>
#include <map>
#include <random>
#include <stdexcept>

#include "Acts/Fitter/GainMatrixSmoother.hpp"
#include "Acts/Fitter/GainMatrixUpdater.hpp"
#include "Acts/Geometry/GeometryID.hpp"
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

#include "ACTFW/Plugins/BField/ScalableBField.hpp"


/**
 * Struct that calls the fitting algorithm to get the result of the fit
 */
namespace {
template <typename Fitter>
struct TrkrFitterFunctionImpl
{
  Fitter fitter;

  TrkrFitterFunctionImpl(Fitter&& f) : fitter(std::move(f)) {}

  FW::TrkrClusterFittingAlgorithm::FitterResult
  operator()(const std::vector<FW::Data::TrkrClusterSourceLink>& sourceLinks,
             const FW::TrackParameters&                  initialParameters,
             const Acts::KalmanFitterOptions<Acts::VoidOutlierFinder>&            options) const
  {
    return fitter.fit(sourceLinks, initialParameters, options);
  };
};
}  // namespace

/**
 * Function that actually makes the fitting function to be used 
 */
FW::TrkrClusterFittingAlgorithm::FitterFunction
FW::TrkrClusterFittingAlgorithm::makeFitterFunction(
    std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry,
    FW::Options::BFieldVariant                    magneticField,
    Acts::Logging::Level                          level)
{
  using Updater  = Acts::GainMatrixUpdater<Acts::BoundParameters>;
  using Smoother = Acts::GainMatrixSmoother<Acts::BoundParameters>;

  /// Return a new instance of the fitter
  return std::visit(
      [trackingGeometry, level](auto&& inputField) -> FitterFunction {
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
        Fitter     fitter(std::move(propagator),
                      Acts::getDefaultLogger("KalmanFitter", level));

        /// Build the fitter function
        return TrkrFitterFunctionImpl<Fitter>(std::move(fitter));
      },
      std::move(magneticField));
}
