add_library(
  ActsExamplesIoObj SHARED
  src/ObjHelper.cpp
  src/ObjSurfaceWriter.cpp
  src/ObjTrackingGeometryWriter.cpp)
target_include_directories(
  ActsExamplesIoObj
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(
  ActsExamplesIoObj
  PUBLIC ActsCore ActsExamplesFramework Threads::Threads)

install(
  TARGETS ActsExamplesIoObj
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
DIRECTORY include/ACTFW DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})