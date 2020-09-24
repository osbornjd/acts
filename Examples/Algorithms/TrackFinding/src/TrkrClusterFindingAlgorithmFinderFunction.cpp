#include "ActsExamples/TrackFinding/TrkrClusterFindingAlgorithm.hpp"

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

    ActsExamples::TrkrClusterFindingAlgorithm::FinderResult
    operator()(
       const std::vector<SourceLink>& sourceLinks,
       const ActsExamples::TrackParameters& initialParameters,
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
ActsExamples::TrkrClusterFindingAlgorithm::FinderFunction
ActsExamples::TrkrClusterFindingAlgorithm::makeFinderFunction(
    std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry,
    Options::BFieldVariant magneticField)
{
  using Updater  = Acts::GainMatrixUpdater;
  using Smoother = Acts::GainMatrixSmoother;

  /// Return a new instance of the finder with the given magnetic field
  /// need to unpack the magnetic field and return the finder
  return std::visit(
      [trackingGeometry](auto&& inputField) -> FinderFunction {
	/// Construct some aliases for the components below
        using InputMagneticField = typename std::decay_t<decltype(inputField)>::element_type;
        using MagneticField      = Acts::SharedBField<InputMagneticField>;
        using Stepper            = Acts::EigenStepper<MagneticField>;
        using Navigator          = Acts::Navigator;
        using Propagator         = Acts::Propagator<Stepper, Navigator>;
	using SourceLinkSelector = Acts::CKFSourceLinkSelector;
        using Finder             = Acts::CombinatorialKalmanFilter
	  <Propagator, Updater, Smoother, SourceLinkSelector>;

        /// Make the components for the fitter
        MagneticField field(std::move(inputField));
        Stepper       stepper(std::move(field));
        Navigator     navigator(trackingGeometry);
        navigator.resolvePassive   = false;
        navigator.resolveMaterial  = true;
        navigator.resolveSensitive = true;
        Propagator propagator(std::move(stepper), std::move(navigator));
        Finder     finder(std::move(propagator));

        /// Build the fitter function
        return TrkrFindingFunctionImpl<Finder>(std::move(finder));
      },
      std::move(magneticField));
}
