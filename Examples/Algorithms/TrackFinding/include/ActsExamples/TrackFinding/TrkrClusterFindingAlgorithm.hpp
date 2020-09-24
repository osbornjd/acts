
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
#include "Acts/TrackFinding/CKFSourceLinkSelector.hpp"
#include "Acts/TrackFinding/CombinatorialKalmanFilter.hpp"
#include "Acts/Geometry/TrackingGeometry.hpp"

#include "ActsExamples/Plugins/BField/ScalableBField.hpp"
#include "ActsExamples/EventData/Track.hpp"
#include "ActsExamples/Framework/BareAlgorithm.hpp"
#include "ActsExamples/Plugins/BField/BFieldOptions.hpp"
#include "ActsExamples/EventData/TrkrClusterSourceLink.hpp"

using SourceLink = ActsExamples::TrkrClusterSourceLink;

namespace ActsExamples {

  class TrkrClusterFindingAlgorithm : public BareAlgorithm
  {

  public:
    using FinderResult = 
      Acts::Result<Acts::CombinatorialKalmanFilterResult<SourceLink>>;
    using CKFOptions = Acts::CombinatorialKalmanFilterOptions<Acts::CKFSourceLinkSelector>;
    using FinderFunction 
      = std::function<FinderResult(const std::vector<SourceLink>&,
				   const ActsExamples::TrackParameters&,
				   const CKFOptions&)>;

    static FinderFunction
      makeFinderFunction(
	 std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry,
	 Options::BFieldVariant magneticField);

    struct Config
    {
      FinderFunction finder;
    };

    TrkrClusterFindingAlgorithm(Config cfg, Acts::Logging::Level lvl);

  private:
    Config m_cfg;

  };


}
