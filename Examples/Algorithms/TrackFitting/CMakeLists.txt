add_library(
  ActsExamplesTrackFitting SHARED
  src/TrackFittingOptions.cpp
  src/SurfaceSortingAlgorithm.cpp
  src/TrackFittingAlgorithm.cpp
  src/TrkrClusterFittingAlgorithmFitterFunction.cpp
  src/TrackFittingAlgorithmFunction.cpp)
target_include_directories(
  ActsExamplesTrackFitting
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(
  ActsExamplesTrackFitting
  PUBLIC
    ActsCore
    ActsExamplesFramework ActsExamplesMagneticField
    Boost::program_options)

install(
  TARGETS ActsExamplesTrackFitting
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ActsExamples DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

