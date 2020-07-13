// This file is part of the Acts project.
//
// Copyright (C) 2018 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "Acts/Geometry/GenericApproachDescriptor.hpp"
#include "Acts/Surfaces/Surface.hpp"
#include "Acts/Utilities/Intersection.hpp"

void Acts::GenericApproachDescriptor::registerLayer(const Layer& lay) {
  // go through the surfaces
  for (auto& sf : m_surfaceCache) {
    auto mutableSf = const_cast<Surface*>(sf);
    mutableSf->associateLayer(lay);
  }
}

Acts::ObjectIntersection<Acts::Surface>
Acts::GenericApproachDescriptor::approachSurface(
    const GeometryContext& gctx, const Vector3D& position,
    const Vector3D& direction, const BoundaryCheck& bcheck) const {
  // the intersection estimates
  std::vector<ObjectIntersection<Surface>> sIntersections;
  sIntersections.reserve(m_surfaceCache.size());
  for (auto& sf : m_surfaceCache) {
    // intersect
    std::cout << "Joe: check surface intersection in approachSurface for surf "<<sf->geoID() << " with type " << sf->type()<<std::endl;
    auto intersection =
        sf->intersectionEstimate(gctx, position, direction, bcheck);
    sIntersections.push_back(ObjectIntersection<Surface>(intersection, sf));
    std::cout<<"Joe: SurfaceIntersection status = " << static_cast<int>(intersection.status) << std::endl;
    
  }
  // Sort them & return the closest
  std::cout << "Joe: sorting surface intersection vector of size " << sIntersections.size() << std::endl;
  std::sort(sIntersections.begin(), sIntersections.end());
  std::cout<<"Joe: GenericApproachDescriptor - returning surface with geoID " 
	   << (*sIntersections.begin()).object->geoID()<<std::endl;
  return (*sIntersections.begin());
}

const std::vector<const Acts::Surface*>&
Acts::GenericApproachDescriptor::containedSurfaces() const {
  return m_surfaceCache;
}

std::vector<const Acts::Surface*>&
Acts::GenericApproachDescriptor::containedSurfaces() {
  return m_surfaceCache;
}
