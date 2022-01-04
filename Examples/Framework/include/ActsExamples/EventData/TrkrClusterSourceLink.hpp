#pragma once

#include "Acts/EventData/Measurement.hpp"
#include "Acts/EventData/MeasurementHelpers.hpp"
#include "Acts/EventData/SourceLink.hpp"
#include "Acts/Geometry/GeometryIdentifier.hpp"

#include "ActsExamples/EventData/GeometryContainers.hpp"
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>

namespace ActsExamples {

/**
 * This class creates an Acts::SourceLink that relates TrkrClusters to the
 * surface they were measured on. The source link is needed for the fitting
 */
class TrkrClusterSourceLink final : public Acts::SourceLink
{
 public:

  /// Instantiate with a hitid, associated surface, and values that actually
  /// make the measurement. Acts requires the surface be available in this class
  TrkrClusterSourceLink(Acts::GeometryIdentifier gid,
			uint64_t cluskey,
			std::shared_ptr<const Acts::Surface> surface,
			Acts::BoundVector loc,
			Acts::BoundMatrix cov)
    : SourceLink(gid) 
    , m_cluskey(cluskey)
    , m_surface(surface)
    , m_geoId(surface->geometryId())
    , m_loc(loc)
    , m_cov(cov)
{
}

  /// Must be default constructible to satisfy SourceLinkConcept
  TrkrClusterSourceLink() : SourceLink{Acts::GeometryIdentifier{}} {}
  TrkrClusterSourceLink(TrkrClusterSourceLink&&)      = default;
  TrkrClusterSourceLink(const TrkrClusterSourceLink&) = default;

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
  Acts::Measurement<Acts::BoundIndices,2> operator*() const
  {
    Acts::ActsVector<2> par;
    Acts::ActsSymMatrix<2> cov = m_cov.topLeftCorner<2,2>();
    std::array<Acts::BoundIndices,2> indices;
    indices[0] = Acts::BoundIndices::eBoundLoc0;
    indices[1] = Acts::BoundIndices::eBoundLoc1;
    par[0] = m_loc(0);
    par[1] = m_loc(1);
    return Acts::Measurement<Acts::BoundIndices,
			     2>
      (*this,
       indices,
       par,
       cov
       );
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

  // Construct a container for TrkrSourceLinks
  using TrkrClusterSourceLinkContainer = GeometryIdMultiset<TrkrClusterSourceLink>;

}

