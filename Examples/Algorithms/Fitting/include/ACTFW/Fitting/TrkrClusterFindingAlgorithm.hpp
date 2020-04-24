
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
#include <Acts/TrackFinder/CKFSourceLinkSelector.hpp>
#include <Acts/TrackFinder/CombinatorialKalmanFilter.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>

#include "ACTFW/Plugins/BField/ScalableBField.hpp"
#include "ACTFW/EventData/Track.hpp"
#include "ACTFW/Framework/BareAlgorithm.hpp"
#include "ACTFW/Plugins/BField/BFieldOptions.hpp"
#include "ACTFW/EventData/TrkrClusterSourceLink.hpp"

using SourceLink = FW::Data::TrkrClusterSourceLink;

namespace FW{

  class TrkrClusterFindingAlgorithm final : public FW::BareAlgorithm
  {

  public:
    using FinderResult = 
      Acts::Result<Acts::CombinatorialKalmanFilterResult<SourceLink>>;

    using FinderFunction 
      = std::function<FinderResult(const std::vector<SourceLink>&,
				   const FW::TrackParameters&,
				   const Acts::CombinatorialKalmanFilterOptions<Acts::VoidBranchStopper>&)>;

    static FinderFunction
      makeFinderFunction(
	 std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry,
	 FW::Options::BFieldVariant magneticField,
	 Acts::Logging::Level level);

    struct Config
    {
      FinderFunction finder;
    };

    TrkrClusterFindingAlgorithm(Config cfg, Acts::Logging::Level lvl);

  private:
    Config m_cfg;

  };


}
