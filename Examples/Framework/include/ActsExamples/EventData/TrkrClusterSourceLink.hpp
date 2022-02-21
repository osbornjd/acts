#pragma once

#include "Acts/EventData/MeasurementHelpers.hpp"
#include "Acts/EventData/SourceLink.hpp"
#include "Acts/Geometry/GeometryIdentifier.hpp"

#include "ActsExamples/EventData/GeometryContainers.hpp"
#include "ActsExamples/EventData/Measurement.hpp"

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
			Acts::BoundVector loc,
			Acts::BoundMatrix cov)
    : SourceLink(gid) 
    , m_cluskey(cluskey)
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
  
  /// Create Acts::Measurement from information in SourceLink
  ActsExamples::Measurement getMeasurement() const
  {
    Acts::ActsVector<2> par;
    Acts::ActsSymMatrix<2> cov = Acts::ActsSymMatrix<2>::Zero();
    cov(0,0) = m_cov(0,0);
    cov(1,1) = m_cov(1,1); 
    std::array<Acts::BoundIndices,2> indices;
    indices[0] = Acts::BoundIndices::eBoundLoc0;
    indices[1] = Acts::BoundIndices::eBoundLoc1;
    par[0] = m_loc(0);
    par[1] = m_loc(1);
    return Acts::Measurement<Acts::BoundIndices, 2>
      (*this, indices, par, cov);
  }
  
  uint64_t cluskey() const
  {
    return m_cluskey;
  }


 private:

  /// Hitindex corresponding to TrkrDefs::cluskey
  uint64_t m_cluskey;
 
  /// Local x and y position for cluster
  Acts::BoundVector m_loc;
  /// Cluster covariance matrix
  Acts::BoundMatrix m_cov;

  /// Equate the cluster keys
  friend constexpr bool
  operator==(const TrkrClusterSourceLink& lhs, const TrkrClusterSourceLink& rhs)
  {
    return lhs.m_cluskey == rhs.m_cluskey;
  }

  friend constexpr bool operator!=(const TrkrClusterSourceLink& lhs,
				   const TrkrClusterSourceLink& rhs) {
    return not(lhs == rhs);
  }

};

  // Construct a container for TrkrSourceLinks
  using TrkrClusterSourceLinkContainer = GeometryIdMultiset<std::reference_wrapper<TrkrClusterSourceLink>>;
  using TrkrClusterSourceLinkAccessor = GeometryIdMultisetAccessor<std::reference_wrapper<TrkrClusterSourceLink>>;
}

