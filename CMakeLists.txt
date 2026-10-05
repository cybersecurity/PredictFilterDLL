cmake_minimum_required(VERSION 3.15)
project(PredictFilterDLL CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen" CACHE PATH "Eigen include directory")

add_library(PredictFilterDLL SHARED
    src/Logger.cpp
    src/OneEuroFilter.cpp
    src/PoseFilter.cpp
    src/Predictor.cpp
    src/PredictFilter.cpp
)
target_include_directories(PredictFilterDLL
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${EIGEN3_INCLUDE_DIR})
target_compile_definitions(PredictFilterDLL PRIVATE PREDICTFILTER_BUILD _USE_MATH_DEFINES)
set_target_properties(PredictFilterDLL PROPERTIES
    OUTPUT_NAME PredictFilterDLL
    PREFIX ""
    CXX_VISIBILITY_PRESET hidden)

if(WIN32)
    target_sources(PredictFilterDLL PRIVATE src/PredictFilterDLL.def)
endif()
if(MINGW)
    target_link_options(PredictFilterDLL PRIVATE -static -s)
endif()

option(PREDICTFILTER_BUILD_TEST "Build the smoke test" ON)
if(PREDICTFILTER_BUILD_TEST)
    add_executable(predict_test test/predict_test.cpp)
    target_link_libraries(predict_test PRIVATE PredictFilterDLL)
    if(MINGW)
        target_link_options(predict_test PRIVATE -static)
    endif()
endif()

if(WIN32 AND PREDICTFILTER_BUILD_TEST)
    # Differential test against the original binary: compare_original.exe <orig.dll> <rebuilt.dll>
    add_executable(compare_original test/compare_original.cpp)
    target_include_directories(compare_original PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if(MINGW)
        target_link_options(compare_original PRIVATE -static)
    endif()
endif()
