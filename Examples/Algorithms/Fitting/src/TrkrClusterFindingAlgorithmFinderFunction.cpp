#include "ACTFW/Fitting/TrkrClusterFindingAlgorithm.hpp"

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

/**
 * Struct that calls the finding algorithm to get the result of the track
 * propagation/finding
 */
namespace {
  template <typename Finder>
  struct TrkrFindingFunctionImpl
  {
    Finder finder;

    TrkrFindingFunctionImpl(Finder&& f) : finder(std::move(f)) {}

    FW::TrkrClusterFindingAlgorithm::FinderResult
    operator()(
       const std::vector<SourceLink>& sourceLinks,
       const FW::TrackParameters&                          initialParameters,
       const Acts::CombinatorialKalmanFilterOptions<Acts::CKFSourceLinkSelector>&       options) const
    {
      /// Call CombinatorialKalmanFilter findTracks
      return finder.findTracks(sourceLinks, initialParameters, options);
    };
  };
}  // namespace

/**
 * Function that actually makes the track finding function to be used 
 */
FW::TrkrClusterFindingAlgorithm::FinderFunction
FW::TrkrClusterFindingAlgorithm::makeFinderFunction(
    std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry,
    FW::Options::BFieldVariant                    magneticField,
    Acts::Logging::Level                          level)
{
  using Updater  = Acts::GainMatrixUpdater<Acts::BoundParameters>;
  using Smoother = Acts::GainMatrixSmoother<Acts::BoundParameters>;

  /// Return a new instance of the finder with the given magnetic field
  /// need to unpack the magnetic field and return the finder
  return std::visit(
      [trackingGeometry, level](auto&& inputField) -> FinderFunction {
	/// Construct some aliases for the components below
        using InputMagneticField = typename std::decay_t<decltype(inputField)>::element_type;
        using MagneticField      = Acts::SharedBField<InputMagneticField>;
        using Stepper            = Acts::EigenStepper<MagneticField>;
        using Navigator          = Acts::Navigator;
        using Propagator         = Acts::Propagator<Stepper, Navigator>;
        using Finder             = Acts::CombinatorialKalmanFilter
	                           <Propagator, Updater, Smoother>;

        /// Make the components for the fitter
        MagneticField field(std::move(inputField));
        Stepper       stepper(std::move(field));
        Navigator     navigator(trackingGeometry);
        navigator.resolvePassive   = false;
        navigator.resolveMaterial  = true;
        navigator.resolveSensitive = true;
        Propagator propagator(std::move(stepper), std::move(navigator));
        Finder     finder(std::move(propagator),
                      Acts::getDefaultLogger("CombinatorialKalmanFilter", level));

        /// Build the fitter function
        return TrkrFindingFunctionImpl<Finder>(std::move(finder));
      },
      std::move(magneticField));
}
