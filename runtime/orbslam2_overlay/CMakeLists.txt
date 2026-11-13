cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

# Paper 2 BSRW validation: expose the g2o realized-work counters without
# enabling the unrelated RWBC controller or V5 semantic-motion extensions.
option(ORB_SLAM2_ENABLE_REALIZED_WORK_COUNTERS
  "Compile realized-LM-work counters into the optimizer hot path" ON)
if(ORB_SLAM2_ENABLE_REALIZED_WORK_COUNTERS)
  add_definitions(-DORB_SLAM2_REALIZED_WORK_COUNTERS=1)
else()
  add_definitions(-DORB_SLAM2_REALIZED_WORK_COUNTERS=0)
endif()

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 ")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC14)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 4 REQUIRED)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED NO_MODULE)
message(STATUS "Eigen3_INCLUDE_DIRS: ${EIGEN3_INCLUDE_DIRS}")
find_package(Pangolin REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
find_package(OpenSSL REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${JSONCPP_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Cacheable library identities keep validation builds inside the isolated
# worktree while allowing their hashes to be frozen explicitly.
set(ORB_SLAM2_G2O_LIBRARY
  "${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so"
  CACHE FILEPATH "Instrumented g2o shared library")
set(ORB_SLAM2_DBOW2_LIBRARY
  "${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so"
  CACHE FILEPATH "DBoW2 shared library")

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/Object.cpp
src/SemanticObject.cc
src/ExperimentTiming.cc
src/FrameTemporalBaseline.cc
src/BSRWSnapshot.cc
src/V5MotionShadow.cc
src/V5MotionState.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORB_SLAM2_DBOW2_LIBRARY}
${ORB_SLAM2_G2O_LIBRARY}
${JSONCPP_LIBRARIES}
OpenSSL::Crypto
)

option(BUILD_BELIEF_TESTS "Build persistent MapPoint belief regression tests" OFF)
if(BUILD_BELIEF_TESTS)
  enable_testing()
  add_executable(test_dynamic_map_filter
    paper2_development/modules/ObjectDynamic/tests/test_dynamic_map_filter.cc)
  target_include_directories(test_dynamic_map_filter PRIVATE
    ${PROJECT_SOURCE_DIR}/paper2_development/modules/ObjectDynamic)
  target_compile_options(test_dynamic_map_filter PRIVATE -UNDEBUG)
  target_link_libraries(test_dynamic_map_filter ${PROJECT_NAME})
  add_test(NAME persistent_mappoint_belief COMMAND test_dynamic_map_filter)

  add_executable(synthetic_belief_conflict
    paper2_development/modules/ObjectDynamic/tests/synthetic_belief_conflict.cc)
  target_include_directories(synthetic_belief_conflict PRIVATE
    ${PROJECT_SOURCE_DIR}/paper2_development/modules/ObjectDynamic)
  target_link_libraries(synthetic_belief_conflict ${PROJECT_NAME})

  add_executable(test_optimizer_measurement_log
    paper2_development/modules/ObjectDynamic/tests/test_optimizer_measurement_log.cc)
  target_compile_options(test_optimizer_measurement_log PRIVATE -UNDEBUG)
  target_link_libraries(test_optimizer_measurement_log ${PROJECT_NAME})
  add_test(NAME optimizer_measurement_log_noninterference
    COMMAND test_optimizer_measurement_log)
endif()

option(BUILD_V5_SHADOW_TESTS "Build V5 shadow matcher unit tests" OFF)
if(BUILD_V5_SHADOW_TESTS)
  enable_testing()
  add_executable(test_v5_shadow_matching
    paper2_development/modules/ObjectDynamic/tests/test_v5_shadow_matching.cc
    src/V5MotionState.cc)
  target_link_libraries(test_v5_shadow_matching ${OpenCV_LIBS})
  add_test(NAME v5_shadow_matching COMMAND test_v5_shadow_matching)
endif()

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

option(BUILD_REALIZED_WORK_TESTS
  "Build the isolated BSRW/g2o instrumentation regression tests" OFF)
if(BUILD_REALIZED_WORK_TESTS)
  enable_testing()
  find_package(PythonInterp 3 REQUIRED)

  add_executable(test_realized_work_g2o
    paper2_development/modules/ObjectDynamic/tests/test_realized_work_g2o.cc)
  target_compile_options(test_realized_work_g2o PRIVATE -UNDEBUG)
  target_link_libraries(test_realized_work_g2o
    ${ORB_SLAM2_G2O_LIBRARY})
  add_test(NAME realized_work_g2o_counters COMMAND test_realized_work_g2o)

  if(ORB_SLAM2_ENABLE_REALIZED_WORK_COUNTERS)
    add_executable(test_realized_work_pose
      paper2_development/modules/ObjectDynamic/tests/test_realized_work_pose.cc)
    target_compile_options(test_realized_work_pose PRIVATE -UNDEBUG)
    target_link_libraries(test_realized_work_pose ${PROJECT_NAME})
    add_test(NAME realized_work_pose_noninterference COMMAND test_realized_work_pose)
    set(REALIZED_WORK_EXPECTED_COUNTERS 1)
  else()
    set(REALIZED_WORK_EXPECTED_COUNTERS 0)
  endif()

  add_test(NAME realized_work_build_capability_banner
    COMMAND ${PYTHON_EXECUTABLE}
      ${PROJECT_SOURCE_DIR}/paper2_development/modules/ObjectDynamic/tests/test_realized_work_build_banner.py
      --binary $<TARGET_FILE:rgbd_tum>
      --active 0 --shadow 1 --counters ${REALIZED_WORK_EXPECTED_COUNTERS})
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

# Optional isolated outputs for supplementary experiments. Frozen release files remain intact.
if(SUPPLEMENT_OUTPUT)
  set_target_properties(ORB_SLAM2 PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${SUPPLEMENT_OUTPUT}/lib")
  set_target_properties(rgbd_tum PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${SUPPLEMENT_OUTPUT}/bin")
endif()
