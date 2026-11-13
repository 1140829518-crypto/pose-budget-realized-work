cmake_minimum_required(VERSION 3.16)
project(bsrw_snapshot_replay LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(BSRW_ORBSLAM2_ROOT "" CACHE PATH
  "Compatible instrumented Paper2/ORB-SLAM2 source tree")
if(NOT BSRW_ORBSLAM2_ROOT)
  message(FATAL_ERROR
    "Set BSRW_ORBSLAM2_ROOT to the compatible instrumented source tree")
endif()
set(BSRW_G2O_LIBRARY
  "${BSRW_ORBSLAM2_ROOT}/build_bsrw_g2o/lib/libg2o.so"
  CACHE FILEPATH "Instrumented g2o shared library")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres 2.2.0 CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
find_package(OpenSSL REQUIRED)

add_library(bsrw_snapshot_io STATIC
  ${BSRW_ORBSLAM2_ROOT}/src/BSRWSnapshot.cc)
target_include_directories(bsrw_snapshot_io PUBLIC
  ${BSRW_ORBSLAM2_ROOT}/include
  ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(bsrw_snapshot_io PUBLIC
  Eigen3::Eigen ${JSONCPP_LIBRARIES} OpenSSL::Crypto)

add_library(bsrw_ceres_common STATIC ceres_replay_common.cc)
target_include_directories(bsrw_ceres_common PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  ${BSRW_ORBSLAM2_ROOT}/include)
target_link_libraries(bsrw_ceres_common PUBLIC bsrw_snapshot_io Ceres::ceres)

add_executable(ceres_snapshot_replay ceres_snapshot_replay.cc)
target_link_libraries(ceres_snapshot_replay PRIVATE bsrw_ceres_common)

add_executable(snapshot_tool snapshot_tool.cc)
target_link_libraries(snapshot_tool PRIVATE bsrw_ceres_common)

add_executable(g2o_snapshot_replay g2o_snapshot_replay.cc)
target_compile_definitions(g2o_snapshot_replay PRIVATE ORB_SLAM2_REALIZED_WORK_COUNTERS=1)
target_include_directories(g2o_snapshot_replay PRIVATE ${BSRW_ORBSLAM2_ROOT})
target_link_libraries(g2o_snapshot_replay PRIVATE
  bsrw_snapshot_io
  ${BSRW_G2O_LIBRARY})

add_executable(g2o_four_round_replay g2o_four_round_replay.cc)
target_compile_definitions(g2o_four_round_replay PRIVATE ORB_SLAM2_REALIZED_WORK_COUNTERS=1)
target_include_directories(g2o_four_round_replay PRIVATE ${BSRW_ORBSLAM2_ROOT})
target_link_libraries(g2o_four_round_replay PRIVATE
  bsrw_snapshot_io
  ${BSRW_G2O_LIBRARY})

add_executable(ceres_four_round_replay ceres_four_round_replay.cc)
target_link_libraries(ceres_four_round_replay PRIVATE bsrw_ceres_common)

add_executable(model_equivalence_check model_equivalence_check.cc)
target_compile_definitions(model_equivalence_check PRIVATE ORB_SLAM2_REALIZED_WORK_COUNTERS=1)
target_include_directories(model_equivalence_check PRIVATE ${BSRW_ORBSLAM2_ROOT})
target_link_libraries(model_equivalence_check PRIVATE
  bsrw_ceres_common
  ${BSRW_G2O_LIBRARY})
