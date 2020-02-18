// This file is part of the Acts project.
//
// Copyright (C) 2020 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "Acts/EventData/TrackParameters.hpp"
#include "Acts/Utilities/Definitions.hpp"
#include "Acts/Utilities/Logger.hpp"
#include "Acts/Utilities/Result.hpp"
#include "Acts/Utilities/Units.hpp"
#include "Acts/Vertexing/AMVFInfo.hpp"
#include "Acts/Vertexing/TrackToVertexIPEstimator.hpp"
#include "Acts/Vertexing/VertexFinderOptions.hpp"

namespace Acts {

using namespace Acts::UnitLiterals;

/// @class AdaptiveMultiVertexFinder
///
/// @brief Implements an iterative vertex finder
///
////////////////////////////////////////////////////////////
///
/// Brief description of the algorithm implemented:
/// TODO
///
////////////////////////////////////////////////////////////
///
/// @tparam vfitter_t Vertex fitter type
/// @tparam sfinder_t Seed finder type
template <typename vfitter_t, typename sfinder_t>
class AdaptiveMultiVertexFinder {
  using Propagator_t = typename vfitter_t::Propagator_t;
  using InputTrack_t = typename vfitter_t::InputTrack_t;

 public:
  /// @struct Config Configuration struct
  struct Config {
    /// @brief Config constructor
    ///
    /// @param fitter The vertex fitter
    /// @param sfinder The seed finder
    /// @param ipEst TrackToVertexIPEstimator
    Config(vfitter_t fitter, sfinder_t sfinder,
           TrackToVertexIPEstimator<InputTrack_t, Propagator_t> ipEst)
        : vertexFitter(std::move(fitter)),
          seedFinder(std::move(sfinder)),
          ipEstimator(std::move(ipEst)) {}

    // Vertex fitter
    vfitter_t vertexFitter;

    /// Vertex seed finder
    sfinder_t seedFinder;

    // TrackToVertexIPEstimator
    TrackToVertexIPEstimator<InputTrack_t, Propagator_t> ipEstimator;

    /// TODO: Update descriptions!
    /** Define a beam constraint for the fit */
    bool useBeamSpotConstraint = true;

    /**
     * When adding a new vertex to the multi vertex fit,
     * only the tracks whose Z at PCA is closer
     * to the seeded by more than this TracksMaxZinterval
     * value are added to this new vertex.
     *
     * Default is 4 mm. If you cut too hard, you cut out
     * the good cases where the seed finder is not
     * reliable, but the fit would be still able to converge
     * towards the right vertex. If you cut too soft, you
     * consider a lot of tracks which just slow down the fit.
     */

    double TracksMaxZinterval = 4_mm;

    /**
     * After having added one vertex to the fit and having
     * performed the MultiVertex fit again, all the tracks
     * which are compatible to the new vertex by more than
     * this maxVertexChi2 (in units of chi2) value are eliminated from the
     * tracks from which still to seed the next vertex.
     *
     */

    double maxVertexChi2 = 18.42;

    /**
     * As a default the realMultiVertex should stay to false (because this is
     * very well tested).
     *
     * If switched to true, all the tracks are considered to be added to the new
     * vertex after this new one is seeded, and not only the ones which are
     * considered as outliers of previous fitted vertices.
     *
     * The presence of a core of tracks the previous vertices are as attached to
     * stabilizes the fit quite drastically. In case of luminosities higher than
     * the low lumi scenario, one should probably to try to switch this on, or,
     * if this doesn't work, decrease the maxVertexChi2 and the
     * cleaningZinterval to lower values.
     */

    bool realMultiVertex = false;

    /*
     * Decides if you want to use the vtxCompatibility() of the track (set to
     * true) or the chi2() (set to false) as an estimate for a track being an
     * outlier or not. The vtxCompatibility() is the default. In case the track
     * refitting is switched on in the AdaptiveMultiVertex fitter, you may want
     * to use the refitted chi2().
     *
     */

    bool useFastCompatibility = true;

    /*
     * Maximum significance on the distance between two vertices
     * to allow merging of two vertices.
     *
     */

    double cutVertexDependence = 3.;

    /*
     * Has to be setup equal to the minimum weight set in the fitter.
     *
     * In the fitting, when a track has a weight lower than this value,
     * the track is not updated during that iteration.
     */

    double minweight = 0.0001;

    /*
     * Maximum amount of iterations allowed for vertex finding.
     *
     * The more vertices you have in the event, the more iterations you have to
     * allow (safe factor: number of expected vertices * 10)
     *
     */

    int maxIterations = 1000;

    /*
     * Fit also single track vertices
     * (could be usefull for example for H-> gamma gamma)\
     *
     */

    bool addSingleTrackVertices = false;

    bool do3dSplitting = false;

    double maximumVertexContamination = 0.5;

    /*
     * Maximum allowed significance of track position to vertex seed
     */
    double tracksMaxSignificance = 5.;

    /*
     * Toggle vertex seed constraint on/off
     */
    bool useSeedConstraint = true;

    // Diagonal constraint covariance entries in case
    // no beamspot constraint is provided
    double looseConstrValue = 1e+8;
    // Default fitQuality for constraint vertex in case no beamspot
    // constraint is provided
    std::pair<double, double> defaultConstrFitQuality{0., -3.};

  };  // Config struct

  struct State {};

  /// @brief Constructor used if InputTrack_t type == BoundParameters
  ///
  /// @param cfg Configuration object
  /// @param logger The logging instance
  template <typename T = InputTrack_t,
            std::enable_if_t<std::is_same<T, BoundParameters>::value, int> = 0>
  AdaptiveMultiVertexFinder(Config& cfg,
                            std::unique_ptr<const Logger> logger =
                                getDefaultLogger("AdaptiveMultiVertexFinder",
                                                 Logging::INFO))
      : m_cfg(std::move(cfg)),
        m_extractParameters([](T params) { return params; }),
        m_logger(std::move(logger)) {}

  /// @brief Constructor for user-defined InputTrack_t type != BoundParameters
  ///
  /// @param cfg Configuration object
  /// @param func Function extracting BoundParameters from InputTrack_t object
  /// @param logger The logging instance
  AdaptiveMultiVertexFinder(Config& cfg,
                            std::function<BoundParameters(InputTrack_t)> func,
                            std::unique_ptr<const Logger> logger =
                                getDefaultLogger("AdaptiveMultiVertexFinder",
                                                 Logging::INFO))
      : m_cfg(std::move(cfg)),
        m_extractParameters(func),
        m_logger(std::move(logger)) {}

  /// @brief Function that performs the adaptive
  /// multi-vertex finding
  ///
  /// @param allTracks Input track collection
  /// @param vFinderOptions Vertex finder options
  ///
  /// @return Vector of all reconstructed vertices
  Result<std::vector<Vertex<InputTrack_t>>> find(
      const std::vector<InputTrack_t>& allTracks,
      const VertexFinderOptions<InputTrack_t>& vFinderOptions) const;

 private:
  /// Configuration object
  Config m_cfg;

  /// @brief Function to extract track parameters,
  /// InputTrack_t objects are BoundParameters by default, function to be
  /// overwritten to return BoundParameters for other InputTrack_t objects.
  ///
  /// @param InputTrack_t object to extract track parameters from
  const std::function<BoundParameters(InputTrack_t)> m_extractParameters;

  /// Logging instance
  std::unique_ptr<const Logger> m_logger;

  /// Private access to logging instance
  const Logger& logger() const { return *m_logger; }

  Result<Vertex<InputTrack_t>> doSeeding(
      const std::vector<InputTrack_t>& trackVector,
      VertexFinderOptions<InputTrack_t>& vFinderOptions) const;
};

}  // namespace Acts

#include "Acts/Vertexing/AdaptiveMultiVertexFinder.ipp"