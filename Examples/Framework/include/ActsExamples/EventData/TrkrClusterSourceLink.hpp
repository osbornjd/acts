#pragma once

#include "Acts/EventData/Measurement.hpp"
#include "Acts/EventData/MeasurementHelpers.hpp"
#include "Acts/EventData/SourceLinkConcept.hpp"
#include "Acts/EventData/detail/fittable_type_generator.hpp"
#include "Acts/Geometry/GeometryIdentifier.hpp"

#include "ActsExamples/EventData/GeometryContainers.hpp"
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>

namespace ActsExamples {

/**
 * This class creates an Acts::SourceLink that relates TrkrClusters to the
 * surface they were measured on. The source link is needed for the fitting
 */
class TrkrClusterSourceLink
{
 public:

  /// Instantiate with a hitid, associated surface, and values that actually
  /// make the measurement. Acts requires the surface be available in this class
  TrkrClusterSourceLink(uint64_t cluskey,
			std::shared_ptr<const Acts::Surface> surface,
			Acts::BoundVector loc,
			Acts::BoundMatrix cov)
    : m_cluskey(cluskey)
    , m_surface(surface)
    , m_geoId(surface->geometryId())
    , m_loc(loc)
    , m_cov(cov)
{
}

  /// Must be default constructible to satisfy SourceLinkConcept
  TrkrClusterSourceLink()                             = default;
  TrkrClusterSourceLink(TrkrClusterSourceLink&&)      = default;
  TrkrClusterSourceLink(const TrkrClusterSourceLink&) = default;

  /// Needs equality operators defined to satisfy SourceLinkConcept
  TrkrClusterSourceLink& operator=(TrkrClusterSourceLink&&)      = default;
  TrkrClusterSourceLink& operator=(const TrkrClusterSourceLink&) = default;

  const Acts::BoundVector location() const
  {
    return m_loc;
  }
  const Acts::BoundMatrix covariance() const
  {
    return m_cov;
  }

  const Acts::GeometryIdentifier geoId() const 
  {
    return m_geoId;
  }

  /// Needs referenceSurface function to satisfy SourceLinkConcept
  const Acts::Surface& referenceSurface() const 
  {
    return *m_surface;
  }

  /// Create Acts::FittableMeasurement from information in SourceLink
  Acts::FittableMeasurement<TrkrClusterSourceLink> operator*() const
  {

    return Acts::Measurement<TrkrClusterSourceLink, 
			     Acts::BoundIndices,
			     Acts::eBoundLoc0,
			     Acts::eBoundLoc1>
      {m_surface,
	  *this,
	  m_cov.topLeftCorner<2, 2>(),
	  m_loc[0],
	  m_loc[1]
	  };
  }

  uint64_t cluskey() const
  {
    return m_cluskey;
  }


private:

  /// Hitindex corresponding to hitID and the corresponding 
  /// surface to which it belongs to
  uint64_t m_cluskey;
  std::shared_ptr<const Acts::Surface> m_surface;
  Acts::GeometryIdentifier m_geoId;

  /// Local x and y position for cluster
  Acts::BoundVector m_loc;
  /// Cluster covariance matrix
  Acts::BoundMatrix m_cov;

  /// Needs equality operator defined to satisfy SourceLinkConcept
  /// Equate the cluster keys
  friend constexpr bool
  operator==(const TrkrClusterSourceLink& lhs, const TrkrClusterSourceLink& rhs)
  {
    return lhs.m_cluskey == rhs.m_cluskey;
  }

};

    /// Ensure that the SourceLink class satisfies SourceLinkConcept conditions
    static_assert(Acts::SourceLinkConcept<TrkrClusterSourceLink>, 
		  "TrkrClusterSourceLink does not fulfill SourceLinkConcept");

  // Construct a container for TrkrSourceLinks
  using TrkrClusterSourceLinkContainer = GeometryIdMultiset<TrkrClusterSourceLink>;

}

