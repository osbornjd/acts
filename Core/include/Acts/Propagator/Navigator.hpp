// This file is part of the Acts project.
//
// Copyright (C) 2016-2018 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <boost/algorithm/string.hpp>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <string>
#include "Acts/Geometry/BoundarySurfaceT.hpp"
#include "Acts/Geometry/Layer.hpp"
#include "Acts/Geometry/TrackingGeometry.hpp"
#include "Acts/Geometry/TrackingVolume.hpp"
#include "Acts/Propagator/ConstrainedStep.hpp"
#include "Acts/Propagator/Propagator.hpp"
#include "Acts/Surfaces/Surface.hpp"
#include "Acts/Utilities/Units.hpp"

namespace Acts {

using namespace Acts::UnitLiterals;

/// @brief struct for the Navigation options that are forwarded to
///        the geometry
///
/// @tparam propagator_state_t Type of the object for navigation state
/// @tparam object_t Type of the object for navigation to check against
template <typename object_t>
struct NavigationOptions {
  /// The navigation direction
  NavigationDirection navDir = forward;

  /// The boundary check directive
  BoundaryCheck boundaryCheck = true;

  // How to resolve the geometry
  /// Always look for sensitive
  bool resolveSensitive = true;
  /// Always look for material
  bool resolveMaterial = true;
  /// always look for passive
  bool resolvePassive = false;

  /// object to check against: at start
  const object_t* startObject = nullptr;
  /// object to check against: at end
  const object_t* endObject = nullptr;

  /// Target surface to exclude
  const Surface* targetSurface = nullptr;

  /// The maximum path limit for this navigation step
  double pathLimit = std::numeric_limits<double>::max();

  /// The overstep tolerance for this navigation step
  /// @note must be negative as it describes overstepping
  /// @todo could be dynamic in the future (pT dependent)
  double overstepLimit = -1_um;

  /// Constructor
  ///
  /// @param nDir Navigation direction prescription
  /// @param bcheck Boundary check for the navigation action
  /// @param sobject Start object to check against
  /// @param eobject End object to check against
  /// @param maxStepLength Maximal step length to check against
  NavigationOptions(NavigationDirection ndir, BoundaryCheck bcheck,
                    bool resolves = true, bool resolvem = true,
                    bool resolvep = false, const object_t* sobject = nullptr,
                    const object_t* eobject = nullptr)
      : navDir(ndir),
        boundaryCheck(std::move(bcheck)),
        resolveSensitive(resolves),
        resolveMaterial(resolvem),
        resolvePassive(resolvep),
        startObject(sobject),
        endObject(eobject),
        pathLimit(ndir * std::numeric_limits<double>::max()),
        overstepLimit(-1_um) {}
};

/// Navigator class
///
/// This is an Actor to be added to the ActorList in order to navigate
/// through the static tracking geometry setup.
///
/// The current navigation stage is cached in the state struct and updated
/// when necessary. If any surface in the extrapolation  flow is hit, it is
/// set to the propagation satate, such that other actors can deal wit it.
/// This navigation actor thus always needs to run first!
/// It does two things: it figures out the order of volumes, layers and
/// surfaces. For each propagation step, the operator() runs, which checks if
/// the current surface (or layer/volume boundary) is reached.
///
/// The current target surface is the surface pointed to by of the iterators
/// for the surfaces, layers or volume boundaries.
/// If a surface is found, the state.navigation.currentSurface
/// pointer is set. This  enables subsequent actors to react. Secondly, this
/// actor uses the ordered  iterators  to figure out which surface, layer or
/// volume boundary is _supposed_ to be hit next. It then sets the maximum
/// step size to the path length found out by straight line intersection. If
/// the state is not on surface, it also  re-computes the step size, to make
/// sure we end up at the desired surface.
///
class Navigator {
 public:
  using Surfaces = std::vector<const Surface*>;
  using SurfaceIter = std::vector<const Surface*>::iterator;

  using NavigationSurfaces = std::vector<SurfaceIntersection>;
  using NavigationSurfaceIter = NavigationSurfaces::iterator;

  using NavigationLayers = std::vector<LayerIntersection>;
  using NavigationLayerIter = NavigationLayers::iterator;

  using NavigationBoundaries = std::vector<BoundaryIntersection>;
  using NavigationBoundaryIter = NavigationBoundaries::iterator;

  using ExternalSurfaces = std::multimap<const Layer*, const Surface*>;

  /// The navigation stage
  enum struct Stage : int {
    undefined = 0,
    surfaceTarget = 1,
    layerTarget = 2,
    boundaryTarget = 3
  };

  /// Constructor with shared tracking geometry
  ///
  /// @param tGeometry The tracking geometry for the navigator
  Navigator(std::shared_ptr<const TrackingGeometry> tGeometry = nullptr)
      : trackingGeometry(std::move(tGeometry)) {}

  /// Tracking Geometry for this Navigator
  std::shared_ptr<const TrackingGeometry> trackingGeometry;

  /// The tolerance used to defined "reached"
  double tolerance = s_onSurfaceTolerance;

  /// Configuration for this Navigator
  /// stop at every sensitive surface (whether it has material or not)
  bool resolveSensitive = true;
  /// stop at every material surface (whether it is passive or not)
  bool resolveMaterial = true;
  /// stop at every surface regardless what it is
  bool resolvePassive = false;

  /// Nested State struct
  ///
  /// It acts as an internal state which is
  /// created for every propagation/extrapolation step
  /// and keep thread-local navigation information
  struct State {
    // Navigation on surface level
    /// the vector of navigation surfaces to work through
    NavigationSurfaces navSurfaces = {};
    /// the current surface iterator of the navigation state
    NavigationSurfaceIter navSurfaceIter = navSurfaces.end();

    // Navigation on layer level
    /// the vector of navigation layers to work through
    NavigationLayers navLayers = {};
    /// the current layer iterator of the navigation state
    NavigationLayerIter navLayerIter = navLayers.end();

    // Navigation on volume level
    /// the vector of boundary surfaces to work through
    NavigationBoundaries navBoundaries = {};
    /// the current boundary iterator of the navigation state
    NavigationBoundaryIter navBoundaryIter = navBoundaries.end();

    /// Externally provided surfaces - these are tried to be hit
    ExternalSurfaces externalSurfaces = {};

    /// Navigation sate: the world volume
    const TrackingVolume* worldVolume = nullptr;

    /// Navigation state: the start volume
    const TrackingVolume* startVolume = nullptr;
    /// Navigation state: the start layer
    const Layer* startLayer = nullptr;
    /// Navigation state: the start surface
    const Surface* startSurface = nullptr;
    /// Navigation state - external state: the current surface
    const Surface* currentSurface = nullptr;
    /// Navigation state: the current volume
    const TrackingVolume* currentVolume = nullptr;
    /// Navigation state: the target volume
    const TrackingVolume* targetVolume = nullptr;
    /// Navigation state: the target layer
    const Layer* targetLayer = nullptr;
    /// Navigation state: the target surface
    const Surface* targetSurface = nullptr;

    /// Indicator for start layer treatment
    bool startLayerResolved = false;
    /// Indicator if the target is reached
    bool targetReached = false;
    /// Navigation state : a break has been detected
    bool navigationBreak = false;
    // The navigation stage (@todo: integrate break, target)
    Stage navigationStage = Stage::undefined;
  };

  /// @brief Navigator status call, will be called in two modes
  ///
  /// (a) It initializes the Navigation stream if start volume is
  ///     not yet defined:
  ///  - initialize the volume
  ///  - establish the start layer and start volume
  ///  - set the current surface to the start surface
  ///
  /// (b) It establishes the currentSurface status during
  ///     the propagation flow, currentSurface can be
  ///  - surfaces still to be handled within a layer
  ///  - layers still to be handled within a volume
  ///  - boundaries still to be handled to exit a volume
  ///
  /// @tparam propagator_state_t is the type of Propagatgor state
  /// @tparam stepper_t is the used type of the Stepper by the Propagator
  ///
  /// @param [in,out] state is the mutable propagator state object
  /// @param [in] stepper Stepper in use
  template <typename propagator_state_t, typename stepper_t>
  void status(propagator_state_t& state, const stepper_t& stepper) const {
    // Check if the navigator is inactive
    if (inactive(state, stepper)) {
      return;
    }

    // Set the navigation stage
    state.navigation.navigationStage = Stage::undefined;

    // Call the navigation helper prior to actual navigation
    debugLog(state, [&] { return std::string("Entering navigator::status."); });

    // (a) Pre-stepping call from propgator
    if (not state.navigation.startVolume) {
      // Initialize and return
      initialize(state, stepper);
      return;
    }

    // Navigator status always starts without current surface
    state.navigation.currentSurface = nullptr;

    // (b) Status call within propagation loop
    // Try finding status of surfaces
    if (status(state, stepper, state.navigation.navSurfaces,
               state.navigation.navSurfaceIter)) {
      debugLog(state,
               [&] { return std::string("Status: in surface handling."); });
      if (state.navigation.currentSurface) {
        debugLog(state, [&] {
          return std::string("On surface: switch forward or release.");
        });
        if (++state.navigation.navSurfaceIter ==
            state.navigation.navSurfaces.end()) {
          // this was the last surface, check if we have layers
          if (!state.navigation.navLayers.empty()) {
            ++state.navigation.navLayerIter;
          } else {
            // no layers, go to boundary
            state.navigation.navigationStage = Stage::boundaryTarget;
            return;
          }
        }
      }
      // Set the navigation stage to surface target
      state.navigation.navigationStage = Stage::surfaceTarget;
      debugLog(state,
               [&] { return std::string("Staying focussed on surface."); });
      // Try finding status of layer
    } else if (status(state, stepper, state.navigation.navLayers,
                      state.navigation.navLayerIter)) {
      debugLog(state,
               [&] { return std::string("Status: in layer handling."); });
      if (state.navigation.currentSurface != nullptr) {
        debugLog(state, [&] {
          return std::string("On layer: update layer information.");
        });
        if (resolveSurfaces(state, stepper)) {
          // Set the navigation stage back to surface handling
          state.navigation.navigationStage = Stage::surfaceTarget;
          return;
        }
      } else {
        // Set the navigation stage to layer target
        state.navigation.navigationStage = Stage::layerTarget;
        debugLog(state,
                 [&] { return std::string("Staying focussed on layer."); });
      }
      // Try finding status of boundaries
    } else if (status(state, stepper, state.navigation.navBoundaries,
                      state.navigation.navBoundaryIter)) {
      debugLog(state,
               [&] { return std::string("Status: in boundary handling."); });

      // Are we on the boundary - then overwrite the stage
      if (state.navigation.currentSurface != nullptr) {
        // Set the navigation stage back to surface handling
        debugLog(state, [&] {
          return std::string("On boundary: update volume information.");
        });
        // We are on a boundary, reset all information
        state.navigation.navSurfaces.clear();
        state.navigation.navSurfaceIter = state.navigation.navSurfaces.end();
        state.navigation.navLayers.clear();
        state.navigation.navLayerIter = state.navigation.navLayers.end();
        // Update volume information
        // get the attached volume information
        auto boundary = state.navigation.navBoundaryIter->object;
        state.navigation.currentVolume = boundary->attachedVolume(
            state.geoContext, stepper.position(state.stepping),
            stepper.direction(state.stepping), state.stepping.navDir);
        // No volume anymore : end of known world
        if (!state.navigation.currentVolume) {
          debugLog(state, [&] {
            return std::string(
                "No more volume to progress to, stopping navigation.");
          });
          // Navigation break & release navigation stepping
          state.navigation.navigationBreak = true;
          stepper.releaseStepSize(state.stepping);
          return;
        } else {
          debugLog(state, [&] { return std::string("Volume updated."); });
          // Forget the bounday information
          state.navigation.navBoundaries.clear();
          state.navigation.navBoundaryIter =
              state.navigation.navBoundaries.end();
        }
      } else {
        // Set the navigation stage back to boundary target
        state.navigation.navigationStage = Stage::boundaryTarget;
        debugLog(state,
                 [&] { return std::string("Staying focussed on boundary."); });
      }
    } else if (state.navigation.currentVolume ==
               state.navigation.targetVolume) {
      debugLog(state, [&] {
        return std::string("No further navigation action, proceed to target.");
      });
      // Set navigation break and release the navigation step size
      state.navigation.navigationBreak = true;
      stepper.releaseStepSize(state.stepping);
    } else {
      debugLog(state, [&] {
        return std::string("Status could not be determined - good luck.");
      });
    }
    return;
  }

  /// @brief Navigator target call
  ///
  /// Call options
  /// (a) there are still surfaces to be resolved: handle those
  /// (b) there no surfaces but still layers to be resolved, handle those
  /// (c) there are no surfaces nor layers to be resolved, handle boundary
  ///
  /// @tparam propagator_state_t is the type of Propagatgor state
  /// @tparam stepper_t is the used type of the Stepper by the Propagator
  ///
  /// @param [in,out] state is the mutable propagator state object
  /// @param [in] stepper Stepper in use
  template <typename propagator_state_t, typename stepper_t>
  void target(propagator_state_t& state, const stepper_t& stepper) const {
    // Check if the navigator is inactive
    std::cout<<"Joe: Check if navigator is active"<<std::endl;
    if (inactive(state, stepper)) {
      std::cout<<"Joe: Target inactive, returning from navigator"<<std::endl;
      return;
    }

    // Call the navigation helper prior to actual navigation
    debugLog(state, [&] { return std::string("Entering navigator::target."); });

    std::cout<<"Joe: initialize target and target volume"<<std::endl;
    // Initialize the target and target volume
    if (state.navigation.targetSurface and not state.navigation.targetVolume) {
      // Find out about the target as much as you can
      std::cout<<"Joe: Initializing target"<<std::endl;
      initializeTarget(state, stepper);
      std::cout<<"Joe: target initialized"<<std::endl;
    }
    
    std::cout<<"Joe: Try targeting surfaces"<<std::endl;
    // Try targeting the surfaces - then layers - then boundaries
    if (state.navigation.navigationStage <= Stage::surfaceTarget and
        targetSurfaces(state, stepper)) {
      std::cout<<"Joe: target set to next surface"<<std::endl;
      debugLog(state,
               [&] { return std::string("Target set to next surface."); });
    } else if (state.navigation.navigationStage <= Stage::layerTarget and
               targetLayers(state, stepper)) {
      std::cout<<"Joe: target set to next layer"<<std::endl;
      debugLog(state, [&] { return std::string("Target set to next layer."); });
    } else if (targetBoundaries(state, stepper)) {
      std::cout<<"Joe: target set to next boundary"<<std::endl;
      debugLog(state,
               [&] { return std::string("Target set to next boundary."); });
    } else {
      std::cout<<"Joe: no further navigation action"<<std::endl;
      debugLog(state, [&] {
        return std::string("No further navigation action, proceed to target.");
      });

      std::cout<<"Joe: set navigation break"<<std::endl;
      // Set navigation break and release the navigation step size
      state.navigation.navigationBreak = true;
      stepper.releaseStepSize(state.stepping);
    }

    std::cout<<"Joe: Reset current surface"<<std::endl;
    // Navigator target always resets the current surface
    state.navigation.currentSurface = nullptr;

    // Return to the propagator
    return;
  }

 private:
  /// --------------------------------------------------------------------
  /// Initialize call - start of propagation
  ///
  /// @tparam propagator_state_t The state type of the propagagor
  /// @tparam stepper_t The type of stepper used for the propagation
  ///
  /// @param [in,out] state is the propagation state object
  /// @param [in] stepper Stepper in use
  ///
  /// @return boolean return triggers exit to stepper
  template <typename propagator_state_t, typename stepper_t>
  void initialize(propagator_state_t& state, const stepper_t& stepper) const {
    // Call the navigation helper prior to actual navigation
    debugLog(state, [&] { return std::string("Initialization."); });

    // Set the world volume if it is not set
    if (not state.navigation.worldVolume) {
      state.navigation.worldVolume = trackingGeometry->highestTrackingVolume();
    }

    // We set the current surface to the start surface
    // for eventual post-update action, e.g. material integration
    // or collection when leaving a surface at the start of
    // an extrapolation process
    state.navigation.currentSurface = state.navigation.startSurface;
    if (state.navigation.currentSurface) {
      debugLog(state, [&] {
        std::stringstream dstream;
        dstream << "Current surface set to start surface ";
        dstream << state.navigation.currentSurface->geoID();
        return dstream.str();
      });
    }
    // Fast Navigation initialization for start condition:
    // - short-cut through object association, saves navigation in the
    // - geometry and volume tree search for the lowest volume
    if (state.navigation.startSurface &&
        state.navigation.startSurface->associatedLayer()) {
      debugLog(state, [&] {
        return std::string("Fast start initialization through association.");
      });
      // assign the current layer and volume by association
      state.navigation.startLayer =
          state.navigation.startSurface->associatedLayer();
      state.navigation.startVolume =
          state.navigation.startLayer->trackingVolume();
      // Set the start volume as current volume
      state.navigation.currentVolume = state.navigation.startVolume;
    } else {
      debugLog(state, [&] {
        return std::string("Slow start initialization through search.");
      });
      // current volume and layer search through global search
      debugLog(state, [&] {
        std::stringstream dstream;
        dstream << "Starting from position "
                << toString(stepper.position(state.stepping));
        dstream << " and direction "
                << toString(stepper.direction(state.stepping));
        return dstream.str();
      });
      state.navigation.startVolume = trackingGeometry->lowestTrackingVolume(
          state.geoContext, stepper.position(state.stepping));
      state.navigation.startLayer =
          state.navigation.startVolume
              ? state.navigation.startVolume->associatedLayer(
                    state.geoContext, stepper.position(state.stepping))
              : nullptr;
      // Set the start volume as current volume
      state.navigation.currentVolume = state.navigation.startVolume;
      if (state.navigation.startVolume) {
        debugLog(state, [&] { return std::string("Start volume resolved."); });
      }
    }
    return;
  }

  /// Status call for test surfaces (surfaces, layers, boundaries)
  /// -------------------------------------------------
  ///
  /// If there are surfaces to be handled, check if the current
  /// state is on the surface
  ///
  /// @tparam propagator_state_t The state type of the propagagor
  /// @tparam stepper_t The type of stepper used for the propagation
  /// @tparam navigation_surfaces_t Type of the propagagor
  /// @tparam navigation_iter_t Type of the navigation iterator
  ///
  /// @param [in,out] state is the propagation state object
  /// @param [in] stepper Stepper in use
  /// @param [in] navSurfaces is the navigation status objects
  /// @param [in] navIter test surface fore the status test
  ///
  /// @return boolean return triggers exit to stepper
  template <typename propagator_state_t, typename stepper_t,
            typename navigation_surfaces_t, typename navigation_iter_t>
  bool status(propagator_state_t& state, const stepper_t& stepper,
              navigation_surfaces_t& navSurfaces,
              const navigation_iter_t& navIter) const {
    // No surfaces, status check will be done on layer
    if (navSurfaces.empty() or navIter == navSurfaces.end()) {
      return false;
    }
    // Take the current surface
    auto surface = navIter->representation;
    // Check if we are at a surface
    // If we are on the surface pointed at by the iterator, we can make
    // it the current one to pass it to the other actors
    auto surfaceStatus =
        stepper.updateSurfaceStatus(state.stepping, *surface, true);
    if (surfaceStatus == Intersection::Status::onSurface) {
      debugLog(state, [&] {
        return std::string("Status Surface successfully hit, storing it.");
      });
      // Set in navigation state, so actors and aborters can access it
      state.navigation.currentSurface = surface;
      if (state.navigation.currentSurface) {
        debugLog(state, [&] {
          std::stringstream dstream;
          dstream << "Current surface set to surface ";
          dstream << state.navigation.currentSurface->geoID();
          return dstream.str();
        });
      }
    }
    // Return a positive status: either on it, or on the way
    return true;
  }

  /// Loop over surface candidates here:
  ///  - if an intersect is  valid but not yet reached
  ///    then return with updated step size
  ///  - if an intersect is not valid, switch to next
  ///
  /// @tparam propagator_state_t The state type of the propagagor
  /// @tparam stepper_t The type of stepper used for the propagation
  ///
  /// @param [in,out] state is the propagation state object
  /// @param [in] stepper Stepper in use
  ///
  /// boolean return triggers exit to stepper
  template <typename propagator_state_t, typename stepper_t>
  bool targetSurfaces(propagator_state_t& state,
                      const stepper_t& stepper) const {
    std::cout<<"Joe: targetting surface"<<std::endl;
    if (state.navigation.navigationBreak) {
      return false;
    }

    // Make sure resolve Surfaces is called on the start layer
    std::cout<<"Joe: resolveSurfaces called on start layer"<<std::endl;
    if (state.navigation.startLayer and
        not state.navigation.startLayerResolved) {
      std::cout<<"Joe: start layer to be resolved"<<std::endl;
      debugLog(state,
               [&] { return std::string("Start layer to be resolved."); });
      // We provide the layer to the resolve surface method in this case
      state.navigation.startLayerResolved = true;
      std::cout<<"Joe: resolveSurfaces in targetSurfaces"<<std::endl;
      bool startResolved =
          resolveSurfaces(state, stepper, state.navigation.startLayer);
      if (not startResolved and
          state.navigation.startLayer == state.navigation.targetLayer) {
	std::cout<<"Joe: start is target layer, nothing left to do"<<std::endl;
        debugLog(state, [&] {
          return std::string("Start is target layer, nothing left to do.");
        });
        // set the navigation break
        state.navigation.navigationBreak = true;
        stepper.releaseStepSize(state.stepping);
      }
      std::cout<<"Joe: returning startResolved = " << startResolved <<" in targetSurfaces"<<std::endl;
      return startResolved;
    }

    // The call that we are on a layer and have not yet resolved the surfaces
    // No surfaces, do not return to stepper
    if (state.navigation.navSurfaces.empty() or
        state.navigation.navSurfaceIter == state.navigation.navSurfaces.end()) {
      debugLog(state, [&] {
        return std::string("No surfaces present, target at layer first.");
      });
      std::cout<<"Joe: no surfaces present, target at layer first"<<std::endl;
      return false;
    }
    std::cout<<"Joe: loop over remiaining navigation surfaces"<<std::endl;
    // Loop over the remaining navigation surfaces
    while (state.navigation.navSurfaceIter !=
           state.navigation.navSurfaces.end()) {
      // Screen output how much is left to try
      debugLog(state, [&] {
        std::stringstream dstream;
        dstream << std::distance(state.navigation.navSurfaceIter,
                                 state.navigation.navSurfaces.end());
        dstream << " out of " << state.navigation.navSurfaces.size();
        dstream << " surfaces remain to try.";
        return dstream.str();
      });
      // Take the surface
      auto surface = state.navigation.navSurfaceIter->object;
      std::cout<<"Joe: next surface candidate will be "<< surface->geoID()<<std::endl;
      // Screen output which surface you are on
      debugLog(state, [&] {
        std::stringstream dstream;
        dstream << "Next surface candidate will be ";
        dstream << surface->geoID();
        return dstream.str();
      });
      // Estimate the surface status
      auto surfaceStatus =
          stepper.updateSurfaceStatus(state.stepping, *surface, true);
      if (surfaceStatus == Intersection::Status::reachable) {
        debugLog(state, [&] {
          std::stringstream dstream;
          dstream << "Surface reachable, step size updated to ";
          dstream << stepper.outputStepSize(state.stepping);
          return dstream.str();
        });
	std::cout<<"Joe: surface reachable, step size updated to "<<stepper.outputStepSize(state.stepping)<<",  returning true"<<std::endl;
        return true;
      }
      ++state.navigation.navSurfaceIter;
      continue;
    }

    // Reached the end of the surface iteration
    if (state.navigation.navSurfaceIter == state.navigation.navSurfaces.end()) {
      debugLog(state, [&] {
        return std::string("Last surface on layer reached, switching layer.");
      });
      // first clear the surface cache
      state.navigation.navSurfaces.clear();
      state.navigation.navSurfaceIter = state.navigation.navSurfaces.end();
      // now switch to the next layer
      ++state.navigation.navLayerIter;
    }
    std::cout<<"Joe: target surface not found, not returning to propagator"<<std::endl;
    // Do not return to the propagator
    return false;
  }

  /// @brief Target layer candidates.
  ///
  /// We are now trying to advance to the next layer (with surfaces)
  /// Check if we are on the representing surface of the layer pointed
  /// at by navLayerIter. If so, we unpack the compatible surfaces
  /// (determined by straight line intersect), and set up the iterator
  /// so that the next status() call will enter the surface
  /// check mode above. If no surfaces are found, we skip the layer.
  /// If we unpack a surface, the step size is set to the path length
  /// to the first surface, as determined by straight line intersect.
  ///
  /// @tparam propagator_state_t The state type of the propagagor
  /// @tparam stepper_t The type of stepper used for the propagation
  ///
  /// @param [in,out] state is the propagation state object
  /// @param [in] stepper Stepper in use
  ///
  /// @return boolean return triggers exit to stepper
  template <typename propagator_state_t, typename stepper_t>
  bool targetLayers(propagator_state_t& state, const stepper_t& stepper) const {
    std::cout<<"Joe: starting targetLayers"<<std::endl;
    if (state.navigation.navigationBreak) {
      return false;
    }

    // if there are no layers, go back to the navigator (not stepper yet)
    if (state.navigation.navLayers.empty()) {
      std::cout<<"Joe: no layers present, resolve volume first"<<std::endl;
      debugLog(state, [&] {
        return std::string("No layers present, resolve volume first.");
      });

      // check if current volume has BVH, or layers
      if (state.navigation.currentVolume->hasBoundingVolumeHierarchy()) {
        // has hierarchy, use that, skip layer resolution
        NavigationOptions<Surface> navOpts(
            state.stepping.navDir, true, resolveSensitive, resolveMaterial,
            resolvePassive, nullptr, state.navigation.targetSurface);
        navOpts.overstepLimit = stepper.overstepLimit(state.stepping);
	std::cout<<"Creating navOpts overstepLimit in targetLayers == "<<navOpts.overstepLimit<<std::endl;
        double opening_angle = 0;

        // Preliminary version of the frustum opening angle estimation.
        // Currently not used (only rays), but will be.

        /*
        Vector3D pos = stepper.position(state.stepping);
        double   mom
            = units::Nat2SI<units::MOMENTUM>(stepper.momentum(state.stepping));
        double   q   = stepper.charge(state.stepping);
        Vector3D dir = stepper.direction(state.stepping);
        Vector3D B   = stepper.getField(state.stepping, pos);
        if (B.squaredNorm() > 1e-9) {
          // ~ non-zero field
          double ir = (dir.cross(B).norm()) * q / mom;
          double s;
          if (state.stepping.navDir == forward) {
            s = state.stepping.stepSize.max();
          } else {
            s = state.stepping.stepSize.min();
          }
          opening_angle = std::atan((1 - std::cos(s * ir)) / std::sin(s * ir));
        }
        debugLog(state, [&] {
          std::stringstream ss;
          ss << std::fixed << std::setprecision(50);
          ss << "Estimating opening angle for frustum nav:" << std::endl;
          ss << "pos: " << pos.transpose() << std::endl;
          ss << "dir: " << dir.transpose() << std::endl;
          ss << "B: " << B.transpose() << " |B|: " << B.norm() << std::endl;
          ss << "step mom: " << stepper.momentum(state.stepping) << std::endl;
          ss << "=> opening angle: " << opening_angle << std::endl;
          return ss.str();
        });
        */

        auto protoNavSurfaces =
            state.navigation.currentVolume->compatibleSurfacesFromHierarchy(
                state.geoContext, stepper.position(state.stepping),
                stepper.direction(state.stepping), opening_angle, navOpts);
        if (!protoNavSurfaces.empty()) {
          // did we find any surfaces?
	  std::cout<<"Joe: found surfaces in targetLayers"<<std::endl;
          // Check: are we on the first surface?
          if (state.navigation.currentSurface == nullptr ||
              state.navigation.currentSurface !=
                  protoNavSurfaces.front().object) {
            // we are not, go on
            state.navigation.navSurfaces = std::move(protoNavSurfaces);
	    std::cout<<"Joe: not on first surface in targetLayers"<<std::endl;
            state.navigation.navSurfaceIter =
                state.navigation.navSurfaces.begin();
            state.navigation.navLayers = {};
            state.navigation.navLayerIter = state.navigation.navLayers.end();
            // The stepper updates the step size ( single / multi component)
            stepper.updateStepSize(state.stepping,
                                   *state.navigation.navSurfaceIter, true);
	    std::cout<<"Joe: updating step size in targetLayers "<<stepper.outputStepSize(state.stepping)<<std::endl;
            debugLog(state, [&] {
              std::stringstream dstream;
              dstream << "Navigation stepSize updated to ";
              dstream << stepper.outputStepSize(state.stepping);
              return dstream.str();
            });
            return true;
          }
        }
      }
      
      if (resolveLayers(state, stepper)) {
        // The layer resolving worked
	std::cout<<"Joe: layer resolving worked"<<std::endl;
        return true;
      }
    }
    std::cout<<"Joe: finished empty nav layers"<<std::endl;
    // loop over the available navigation layer candiates
    while (state.navigation.navLayerIter != state.navigation.navLayers.end()) {
      // The layer surface
      auto layerSurface = state.navigation.navLayerIter->representation;
      // We are on the layer
      if (state.navigation.currentSurface == layerSurface) {
        debugLog(state, [&] {
          return std::string("We are on a layer, resolve Surfaces.");
        });
	std::cout<<"Joe: on a layer, run resolveSurfaces"<<std::endl;
        // If you found surfaces return to the propagator
        if (resolveSurfaces(state, stepper)) {
	  std::cout<<"Joe: resolved a surface from layer, return true"<<std::endl;
          return true;
        } else {
          // Try the next one
          ++state.navigation.navLayerIter;
          continue;
        }
      }
      // Try to step towards it
      auto layerStatus =
          stepper.updateSurfaceStatus(state.stepping, *layerSurface, true);
      if (layerStatus == Intersection::Status::reachable) {
	std::cout<<"Joe: layer reachable, step size updated to "<<
	  stepper.outputStepSize(state.stepping)<<std::endl;
	debugLog(state, [&] {
	    std::stringstream dstream;
	    dstream << "Layer reachable, step size updated to ";
	    dstream << stepper.outputStepSize(state.stepping);
	    return dstream.str();
	  });
        return true;
      }
      debugLog(state, [&] {
        return std::string("Layer intersection not valid, skipping it.");
      });
      ++state.navigation.navLayerIter;
    }
    std::cout<<"Joe: finished layer iteration"<<std::endl;
    // Re-initialize target at last layer, only in case it is the target volume
    // This avoids a wrong target volume estimation
    if (state.navigation.currentVolume == state.navigation.targetVolume) {
      std::cout<<"Joe: reinitializing target in targetLayers"<<std::endl;
      initializeTarget(state, stepper);
    }
    // Screen output
    debugLog(state, [&] {
      std::stringstream dstream;
      dstream << "Last layer";
      if (state.navigation.currentVolume == state.navigation.targetVolume) {
        dstream << " (final volume) done, proceed to target.";
      } else {
        dstream << " done, target volume boundary.";
      }
      return dstream.str();
    });
    std::cout<<"Joe: set the navigation break in targetLayers"<<std::endl;
    // Set the navigation break if necessary
    state.navigation.navigationBreak =
        (state.navigation.currentVolume == state.navigation.targetVolume);
    return false;
  }

  /// Navigation through volumes
  /// -------------------------------------------------
  ///
  /// This is the boundary check routine. If the code above set up the
  /// boundary surface iterator, we advance through them here. If we are on
  /// the boundary surface, we set the current surface to the boundary
  /// surface, and get the volume pointed at by the boundary surface.  Next
  /// we unpack the layers from that volume. If the volume contains layers
  /// we set the step size to the straight line path length to the first
  /// layer.  If we don't find a next volume, the navigationBreak
  /// indicator is set.  This ends the navigation. Finally, the boundary
  /// iterator is cleared, so that the subsequent call goes back to
  /// the layer iteration logic.
  ///
  /// If we are not on the current boundary surface, we try the next one.
  /// The iterator is advanced and the step size is set. If no straight
  /// line intersect is found, the boundary surface is skipped.
  /// If we are out of boundary surfaces, the navigation is terminated.
  ///
  /// @tparam propagator_state_t The state type of the propagagor
  /// @tparam stepper_t The type of stepper used for the propagation
  ///
  /// @param [in,out] state is the propagation state object
  /// @param [in] stepper Stepper in use
  ///
  /// boolean return triggers exit to stepper
  template <typename propagator_state_t, typename stepper_t>
  bool targetBoundaries(propagator_state_t& state,
                        const stepper_t& stepper) const {
    if (state.navigation.navigationBreak) {
      return false;
    }

    if (!state.navigation.currentVolume) {
      debugLog(state, [&] {
        return std::string(
            "No sufficient information to resolve boundary, "
            "stopping navigation.");
      });
      stepper.releaseStepSize(state.stepping);
      return false;
    } else if (state.navigation.currentVolume ==
               state.navigation.targetVolume) {
      debugLog(state, [&] {
        return std::string(
            "In target volume: no need to resolve boundary, "
            "stopping navigation.");
        state.navigation.navigationBreak = true;
        stepper.releaseStepSize(state.stepping);
      });
      return true;
    }

    // Re-initialize target at volume boundary
    initializeTarget(state, stepper);

    // Helper function to find boundaries
    auto findBoundaries = [&]() -> bool {
      // The navigation options
      NavigationOptions<Surface> navOpts(state.stepping.navDir, true);
      navOpts.pathLimit =
          state.stepping.stepSize.value(ConstrainedStep::aborter);
      navOpts.overstepLimit = stepper.overstepLimit(state.stepping);

      // Exclude the current surface in case it's a boundary
      navOpts.startObject = state.navigation.currentSurface;
      debugLog(state, [&] {
        std::stringstream ss;
        ss << "Try to find boundaries, we are at: "
           << stepper.position(state.stepping).transpose()
           << ", dir: " << stepper.direction(state.stepping).transpose();
        return ss.str();
      });
      // Evaluate the boundary surfaces
      state.navigation.navBoundaries =
          state.navigation.currentVolume->compatibleBoundaries(
              state.geoContext, stepper.position(state.stepping),
              stepper.direction(state.stepping), navOpts);
      // The number of boundary candidates
      debugLog(state, [&] {
        std::stringstream dstream;
        dstream << state.navigation.navBoundaries.size();
        dstream << " boundary candidates found at path(s): ";
        for (auto& bc : state.navigation.navBoundaries) {
          dstream << bc.intersection.pathLength << "  ";
        }
        return dstream.str();
      });
      // Set the begin iterator
      state.navigation.navBoundaryIter = state.navigation.navBoundaries.begin();
      if (not state.navigation.navBoundaries.empty()) {
        // Set to the first and return to the stepper
        stepper.updateStepSize(state.stepping,
                               *state.navigation.navBoundaryIter, true);
        debugLog(state, [&] {
          std::stringstream dstream;
          dstream << "Navigation stepSize updated to ";
          dstream << stepper.outputStepSize(state.stepping);
          return dstream.str();
        });
	std::cout<<"Joe: targetBoundaries stepsize updated to "<<stepper.outputStepSize(state.stepping)<<std::endl;
        return true;
      }
      return false;
    };

    // No boundaries are assigned yet, find them
    if (state.navigation.navBoundaries.empty() and findBoundaries()) {
      return true;
    }

    // Loop over the boundary surface
    while (state.navigation.navBoundaryIter !=
           state.navigation.navBoundaries.end()) {
      // That is the current boundary surface
      auto boundarySurface = state.navigation.navBoundaryIter->representation;
      // Step towards the boundary surfrace
      auto boundaryStatus =
          stepper.updateSurfaceStatus(state.stepping, *boundarySurface, true);
      if (boundaryStatus == Intersection::Status::reachable) {
        debugLog(state, [&] {
          std::stringstream dstream;
          dstream << "Boundary reachable, step size updated to ";
          dstream << stepper.outputStepSize(state.stepping);
          return dstream.str();
        });
	std::cout<<"Joe: boundary reachable, step size updated to "<<stepper.outputStepSize(state.stepping)<<std::endl;
        return true;
      } else {
        debugLog(state, [&] {
          std::stringstream dstream;
          dstream << "Boundary ";
          dstream << std::distance(state.navigation.navBoundaryIter,
                                   state.navigation.navBoundaries.end());
          dstream << " out of " << state.navigation.navBoundaries.size();
          dstream << " not reachable anymore, switching to next.";
          return dstream.str();
        });
      }
      // Increase the iterator to the next one
      ++state.navigation.navBoundaryIter;
    }
    // We have to leave the volume somehow, so try again
    state.navigation.navBoundaries.clear();
    debugLog(state, [&] {
      return std::string("Boundary navigation lost, re-targetting.");
    });
    if (findBoundaries()) {
      return true;
    }

    // Tried our best, but couldn't do anything
    return false;
  }

  /// --------------------------------------------------------------------
  /// Navigation (re-)initialisation for the target
  ///
  /// This is only called a few times every propagation/extrapolation
  ///
  /// ---------------------------------------------------------------------
  ///
  /// As a straight line estimate can lead you to the wrong destination
  /// Volume, this will be called at:
  /// - initialization
  /// - attempted volume switch
  /// Target finding by association will not be done again
  ///
  /// @tparam propagator_state_t The state type of the propagagor
  /// @tparam stepper_t The type of stepper used for the propagation
  ///
  /// @param [in,out] state is the propagation state object
  /// @param [in] stepper Stepper in use
  ///
  /// boolean return triggers exit to stepper
  template <typename propagator_state_t, typename stepper_t>
  void initializeTarget(propagator_state_t& state,
                        const stepper_t& stepper) const {
    if (state.navigation.targetVolume and
        state.stepping.pathAccumulated == 0.) {
      debugLog(state, [&] {
        return std::string("Re-initialzing cancelled as it is the first step.");
      });
      std::cout<<"Joe: Re-initializing cancelled as it is the first step"<<std::endl;
      return;
    }
    std::cout<<"Joe: Check targetSurface layer and volume"<<std::endl;
    // Fast Navigation initialization for target:
    if (state.navigation.targetSurface &&
        state.navigation.targetSurface->associatedLayer() &&
        !state.navigation.targetVolume) {
      std::cout<<"Joe: checked with geoid : "<<state.navigation.targetSurface->geoID()<<std::endl;
      debugLog(state, [&] {
        return std::string("Fast target initialization through association.");
      });
      debugLog(state, [&] {
        std::stringstream dstream;
        dstream << "Target surface set to ";
        dstream << state.navigation.targetSurface->geoID();
        return dstream.str();
      });

      std::cout<<"Joe: assign target volume and surface"<<std::endl;
      // assign the target volume and the target surface
      state.navigation.targetLayer =
          state.navigation.targetSurface->associatedLayer();
      state.navigation.targetVolume =
          state.navigation.targetLayer->trackingVolume();
    } else if (state.navigation.targetSurface) {
      std::cout<<"Joe: only targetSurface exists"<<std::endl;
      // screen output that you do a re-initialization
      if (state.navigation.targetVolume) {
        debugLog(state, [&] {
          return std::string("Re-initialization of target volume triggered.");
        });
      }
      // Slow navigation initialization for target:
      // target volume and layer search through global search
      std::cout<<"Joe: re-initialize target volume"<<std::endl;
      auto targetIntersection = state.navigation.targetSurface->intersect(
          state.geoContext, stepper.position(state.stepping),
          state.stepping.navDir * stepper.direction(state.stepping), false);
      if (targetIntersection) {
	std::cout << "Joe: Target estimate position (";
	std::cout << targetIntersection.intersection.position.x() << ", ";
	std::cout << targetIntersection.intersection.position.y() << ", ";
	std::cout << targetIntersection.intersection.position.z() << ")";
        debugLog(state, [&] {
          std::stringstream dstream;
          dstream << "Target estimate position (";
          dstream << targetIntersection.intersection.position.x() << ", ";
          dstream << targetIntersection.intersection.position.y() << ", ";
          dstream << targetIntersection.intersection.position.z() << ")";
          return dstream.str();
        });
        /// get the target volume from the intersection
	std::cout<<"Joe: Get target volume from intersection"<<std::endl;
        auto tPosition = targetIntersection.intersection.position;
        state.navigation.targetVolume =
            trackingGeometry->lowestTrackingVolume(state.geoContext, tPosition);
        state.navigation.targetLayer =
            state.navigation.targetVolume
                ? state.navigation.targetVolume->associatedLayer(
                      state.geoContext, tPosition)
                : nullptr;
        if (state.navigation.targetVolume) {
	  std::cout<<"Joe: target volume estimated "<<state.navigation.targetVolume->volumeName()<<std::endl;
          debugLog(state, [&] {
            std::stringstream dstream;
            dstream << "Target volume estimated : ";
            dstream << state.navigation.targetVolume->volumeName();
            return dstream.str();
          });
        }
      }
    }
    std::cout<<"Joe: returning from initializeTarget"<<std::endl;
  }

  /// @brief Resolve the surfaces of this layer
  ///
  /// @tparam propagator_state_t The state type of the propagagor
  /// @tparam stepper_t The type of stepper used for the propagation
  ///
  /// @param [in,out] state is the propagation state object
  /// @param [in] stepper Stepper in use
  /// @param [in] cLayer is the already assigned current layer, e.g. start layer
  ///
  /// boolean return triggers exit to stepper
  template <typename propagator_state_t, typename stepper_t>
  bool resolveSurfaces(propagator_state_t& state, const stepper_t& stepper,
                       const Layer* cLayer = nullptr) const {
    // get the layer and layer surface
    auto layerSurface = cLayer ? state.navigation.startSurface
                               : state.navigation.navLayerIter->representation;
    std::cout<<"Joe: layerSurface is " << layerSurface->geoID()<<std::endl;
    auto navLayer = cLayer ? cLayer : state.navigation.navLayerIter->object;
    // are we on the start layer
    bool onStart = (navLayer == state.navigation.startLayer);
    auto startSurface = onStart ? state.navigation.startSurface : layerSurface;
    // Use navigation parameters and NavigationOptions
    NavigationOptions<Surface> navOpts(
        state.stepping.navDir, true, resolveSensitive, resolveMaterial,
        resolvePassive, startSurface, state.navigation.targetSurface);
    // Check the limit
    navOpts.pathLimit = state.stepping.stepSize.value(ConstrainedStep::aborter);
    std::cout<<"Joe: navOpts path limit " << navOpts.pathLimit<<std::endl;
    // No overstepping on start layer, otherwise ask the stepper
    navOpts.overstepLimit = (cLayer != nullptr)
                                ? s_onSurfaceTolerance
                                : stepper.overstepLimit(state.stepping);
    std::cout<<"Joe: navOpts overstepLimit = "<<navOpts.overstepLimit<<std::endl;
    // get the surfaces
    std::cout<<"Joe: requesting compatible surface 1091 - " << state.navigation.currentVolume->volumeName()<<std::endl;
    state.navigation.navSurfaces = navLayer->compatibleSurfaces(
        state.geoContext, stepper.position(state.stepping),
        stepper.direction(state.stepping), navOpts);
    // the number of layer candidates
    if (!state.navigation.navSurfaces.empty()) {
      std::cout<<"Joe: number of compatible surfaces = " << state.navigation.navSurfaces.size() << std::endl;
      for(auto& sfc : state.navigation.navSurfaces) {
	std::cout<<"Nav surface : "<<sfc.intersection.pathLength <<"  and position "<<sfc.intersection.position<<std::endl;
      }
      debugLog(state, [&] {
        std::stringstream dstream;
        dstream << state.navigation.navSurfaces.size();
        dstream << " surface candidates found at path(s): ";
        for (auto& sfc : state.navigation.navSurfaces) {
          dstream << sfc.intersection.pathLength << "  ";
        }
        return dstream.str();
      });
      // set the iterator
      state.navigation.navSurfaceIter = state.navigation.navSurfaces.begin();
      // The stepper updates the step size ( single / multi component)
      stepper.updateStepSize(state.stepping, *state.navigation.navSurfaceIter,
                             true);
      std::cout<<"Joe: navigation stepsize updated to "<<stepper.outputStepSize(state.stepping)<<std::endl;
      debugLog(state, [&] {
        std::stringstream dstream;
        dstream << "Navigation stepSize updated to ";
        dstream << stepper.outputStepSize(state.stepping);
        return dstream.str();
      });
      return true;
    }
    std::cout<<"Joe: No surface candidates found"<<std::endl;
    state.navigation.navSurfaceIter = state.navigation.navSurfaces.end();
    debugLog(state,
             [&] { return std::string("No surface candidates found."); });
    return false;
  }

  /// Navigation through layers
  /// -------------------------------------------------
  ///
  /// Resolve layers.
  ///
  /// This initializes the layer candidates when starting
  /// or when entering a new volume
  ///
  /// @tparam propagator_state_t The state type of the propagagor
  /// @tparam stepper_t The type of stepper used for the propagation
  ///
  /// @param [in,out] state is the propagation state object
  /// @param [in] stepper Stepper in use
  ///
  /// @return boolean return triggers exit to stepper
  template <typename propagator_state_t, typename stepper_t>
  bool resolveLayers(propagator_state_t& state,
                     const stepper_t& stepper) const {
    std::cout<<"Joe: starting resolveLayers"<<std::endl;
    debugLog(state,
             [&] { return std::string("Searching for compatible layers."); });

    // Check if we are in the start volume
    auto startLayer =
        (state.navigation.currentVolume == state.navigation.startVolume)
            ? state.navigation.startLayer
            : nullptr;
    // Create the navigation options
    // - and get the compatible layers, start layer will be excluded
    NavigationOptions<Layer> navOpts(state.stepping.navDir, true,
                                     resolveSensitive, resolveMaterial,
                                     resolvePassive, startLayer, nullptr);
    // Set also the target surface
    navOpts.targetSurface = state.navigation.targetSurface;
    navOpts.pathLimit = state.stepping.stepSize.value(ConstrainedStep::aborter);
    navOpts.overstepLimit = stepper.overstepLimit(state.stepping);
    std::cout<<"Joe: resolveLayers path and overstep limit == "<<navOpts.pathLimit<<"  "<<navOpts.overstepLimit<<std::endl;
    // Request the compatible layers
    state.navigation.navLayers =
        state.navigation.currentVolume->compatibleLayers(
            state.geoContext, stepper.position(state.stepping),
            stepper.direction(state.stepping), navOpts);
    std::cout<<"Joe: request compatible layers 1161 - " << state.navigation.currentVolume->volumeName() << std::endl;
    // Layer candidates have been found
    if (!state.navigation.navLayers.empty()) {
      // Screen output where they are
      std::cout<<"Joe: layer candidate size "<<state.navigation.navLayers.size()<<std::endl;
      for(auto& lc : state.navigation.navLayers){
	std::cout<<"Joe: layer candidates "<<lc.intersection.pathLength<<"  "<<lc.intersection.position<<std::endl;
      }
      debugLog(state, [&] {
        std::stringstream dstream;
        dstream << state.navigation.navLayers.size();
        dstream << " layer candidates found at path(s): ";
        for (auto& lc : state.navigation.navLayers) {
          dstream << lc.intersection.pathLength << "  ";
        }
        return dstream.str();
      });
      // Set the iterator to the first
      state.navigation.navLayerIter = state.navigation.navLayers.begin();
      // Setting the step size towards first
      if (state.navigation.startLayer &&
          state.navigation.navLayerIter->object !=
              state.navigation.startLayer) {
	std::cout<<"Joe: target at layer"<<std::endl;
        debugLog(state, [&] { return std::string("Target at layer."); });
        // The stepper updates the step size ( single / multi component)
        stepper.updateStepSize(state.stepping, *state.navigation.navLayerIter,
                               true);
	std::cout<<"Joe: navigation stepsize updated to in resolveLayers "<<stepper.outputStepSize(state.stepping)<<std::endl;
        debugLog(state, [&] {
          std::stringstream dstream;
          dstream << "Navigation stepSize updated to ";
          dstream << stepper.outputStepSize(state.stepping);
          return dstream.str();
        });
        return true;
      }
    }

    // Set the iterator to the end of the list
    state.navigation.navLayerIter = state.navigation.navLayers.end();
    std::cout<<"Joe: no compatible layer candidates found"<<std::endl;
    // Screen output - no layer candidates found
    debugLog(state, [&] {
      return std::string("No compatible layer candidates found.");
    });
    // Release the step size
    stepper.releaseStepSize(state.stepping);
    return false;
  }

  /// --------------------------------------------------------------------
  /// Inactive
  ///
  /// This checks if a navigation break had been triggered or navigator
  /// is misconfigured
  ///
  /// @tparam propagator_state_t The state type of the propagagor
  /// @tparam stepper_t The type of stepper used for the propagation
  ///
  /// @param [in,out] state is the propagation state object
  /// @param [in] stepper Stepper in use
  ///
  /// boolean return triggers exit to stepper
  template <typename propagator_state_t, typename stepper_t>
  bool inactive(propagator_state_t& state, const stepper_t& stepper) const {
    // Void behavior in case no tracking geometry is present
    if (!trackingGeometry) {
      std::cout<<"Joe: No tracking geometry object! Can't navigate"<<std::endl;
      return true;
    }
    // turn the navigator into void when you are intructed to do nothing
    if (!resolveSensitive && !resolveMaterial && !resolvePassive) {
      std::cout<<"Joe: No sensitive material! Can't navigate"<<std::endl;
      return true;
    }

    // --------------------------------------------------------------------
    // Navigation break handling
    // This checks if a navigation break had been triggered:
    // - If so & the target exists or was hit - it simply returns
    // - If a target exists and was not yet hit, it checks for it
    // -> return is always to the stepper
    if (state.navigation.navigationBreak) {
      // target exists and reached, or no target exists
      if (state.navigation.targetReached || !state.navigation.targetSurface) {
	std::cout<<"Joe: target exists and reached, or no target"<<std::endl;
	return true;
      }
      std::cout<<"Joe: Update surface status"<<std::endl;
      auto targetStatus = stepper.updateSurfaceStatus(
          state.stepping, *state.navigation.targetSurface, true);
      // the only advance could have been to the target
      if (targetStatus == Intersection::Status::onSurface) {
        // set the target surface

        state.navigation.currentSurface = state.navigation.targetSurface;
	std::cout<<"Joe: current surface set to target surface with "<<state.navigation.currentSurface->geoID()<<std::endl;
        debugLog(state, [&] {
          std::stringstream dstream;
          dstream << "Current surface set to target surface ";
          dstream << state.navigation.currentSurface->geoID();
          return dstream.str();
        });
        return true;
      }
    }
    std::cout<<"Joe: target surface not successfully initialized"<<std::endl;
    return false;
  }

  /// The private navigation debug logging
  ///
  /// It needs to be fed by a lambda function that returns a string,
  /// that guarantees that the lambda is only called in the
  /// state.options.debug == true
  /// case in order not to spend time when not needed.
  ///
  /// @tparam propagator_state_t Type of the propagator state
  ///
  /// @param[in,out] state the propagator state for the debug flag,
  ///      prefix and length
  /// @param logAction is a callable function that returns a streamable object
  template <typename propagator_state_t>
  void debugLog(propagator_state_t& state,
                const std::function<std::string()>& logAction) const {
    if (state.options.debug) {
      std::string vName = "No Volume";
      if (state.navigation.currentVolume) {
        vName = state.navigation.currentVolume->volumeName();
      }
      std::vector<std::string> lines;
      std::string input = logAction();
      boost::split(lines, input, boost::is_any_of("\n"));
      for (const auto& line : lines) {
        std::stringstream dstream;
        dstream << ">>>" << std::setw(state.options.debugPfxWidth) << vName
                << " | ";
        dstream << std::setw(state.options.debugMsgWidth) << line << '\n';
        state.options.debugString += dstream.str();
      }
    }
  }
};

}  // namespace Acts
