// This file is part of the Acts project.
//
// Copyright (C) 2020 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "Acts/Vertexing/VertexFitterOptions.hpp"
#include "Acts/Vertexing/VertexingError.hpp"

template <typename vfitter_t, typename sfinder_t>
auto Acts::AdaptiveMultiVertexFinder<vfitter_t, sfinder_t>::find(
    const std::vector<InputTrack_t>& allTracks,
    const VertexFinderOptions<InputTrack_t>& vFinderOptions) const
    -> Result<std::vector<Vertex<InputTrack_t>>> {
  if (allTracks.empty()) {
    return VertexingError::EmptyInput;
  }

  // Create copy of finder options, will be modified after seeding
  // to set the correct vertex constraint
  VertexFinderOptions<InputTrack_t> finderOptions = vFinderOptions;

  // Original tracks
  const std::vector<InputTrack_t>& origTracks = allTracks;
  // Tracks for seeding
  // Note: Remains to be investigated if another container (e.g. std::list)
  // or also std::vector<InputTrack_t*> is a faster option since erasures
  // of tracks is quite expensive with std::vector.
  // std::vector<InputTrack_t*> would however also come with an overhead
  // since m_cfg.vertexFitter.fit and m_cfg.seedFinder.find take
  // vector<InputTrack_t> and hence a lot of copying would be required.
  // Maybe use std::vector<InputTrack_t*> and adapt fit accordingly to
  // also take pointers to tracks instead of the track object.
  std::vector<InputTrack_t> seedTracks = allTracks;

  // Construct the vertex fitter options from vertex finder options
  VertexFitterOptions<InputTrack_t> vFitterOptions(
      finderOptions.geoContext, finderOptions.magFieldContext,
      finderOptions.vertexConstraint);

  std::vector<Vertex<InputTrack_t>> allVertices;

  int iteration = 0;
  while (((m_cfg.addSingleTrackVertices && seedTracks.size() > 0) ||
          ((!m_cfg.addSingleTrackVertices) && seedTracks.size() > 1)) &&
         iteration < m_cfg.maxIterations) {
    // Retrieve seed vertex from all remaining seedTracks
    auto seedRes = doSeeding(seedTracks, finderOptions);
    if (!seedRes.ok()) {
      return seedRes.error();
    }
    Vertex<InputTrack_t> vtxCandidate = *seedRes;

    if (vtxCandidate.position().z() == 0.) {
      // No seed found anymore, break and stop primary vertex finding
      break;
    }

    VertexInfo<InputTrack_t> vtxCandidateInfo(finderOptions.vertexConstraint,
                                              vtxCandidate.fullPosition());
  }

  return allVertices;
}

template <typename vfitter_t, typename sfinder_t>
auto Acts::AdaptiveMultiVertexFinder<vfitter_t, sfinder_t>::doSeeding(
    const std::vector<InputTrack_t>& trackVector,
    VertexFinderOptions<InputTrack_t>& vFinderOptions) const
    -> Result<Vertex<InputTrack_t>> {
  // Run seed finder
  auto seedRes = m_cfg.seedFinder.find(trackVector, vFinderOptions);

  if (!seedRes.ok()) {
    return seedRes.error();
  }

  Vertex<InputTrack_t> seedVertex = (*seedRes).back();

  if (m_cfg.useBeamSpotConstraint) {
    if (m_cfg.useSeedConstraint) {
      vFinderOptions.vertexConstraint.setFullPosition(
          seedVertex.fullPosition());
      vFinderOptions.vertexConstraint.setFullCovariance(
          seedVertex.fullCovariance());
    }
  } else {
    vFinderOptions.vertexConstraint.setFullPosition(seedVertex.fullPosition());
    vFinderOptions.vertexConstraint.setFullCovariance(
        SpacePointSymMatrix::Identity() * m_cfg.looseConstrValue);
    vFinderOptions.vertexConstraint.setFitQuality(
        m_cfg.defaultConstrFitQuality);
  }

  return seedVertex;
}