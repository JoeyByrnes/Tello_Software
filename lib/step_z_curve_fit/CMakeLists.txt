######################################################################
# CMakeLists.txt generated for component step_z_curve_fit
# Product type: static library
######################################################################
cmake_minimum_required(VERSION 3.12)
project(step_z_curve_fit)
if(DEFINED ENV{CMAKE_EXPORT_COMPILE_COMMANDS} AND NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS $ENV{CMAKE_EXPORT_COMPILE_COMMANDS})
endif()


######################################################################
## Path variables
######################################################################
get_filename_component(START_DIR ../../.. ABSOLUTE)
set(MATLAB_ROOT C:/Program\ Files/MATLAB/R2023a CACHE PATH "")
if(${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    set(BUILDING_IN_SOURCE_DIR TRUE)
    set(BINARY_START_DIR "${START_DIR}")
else()
    set(BUILDING_IN_SOURCE_DIR FALSE)
    set(BINARY_START_DIR "${CMAKE_BINARY_DIR}")
endif()
if(WIN32 AND NOT MINGW)
    set(SYSLIB_PREFIX "lib")
else()
    set(SYSLIB_PREFIX "")
endif()

######################################################################
## System libraries
######################################################################
find_library(FOUND_LIBM m NO_SYSTEM_ENVIRONMENT_PATH PATHS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES}
    ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})
find_library(FOUND_LIBSTDCXX stdc++ NO_SYSTEM_ENVIRONMENT_PATH PATHS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES}
    ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})

######################################################################
## Target definition and commands
######################################################################
add_library(step_z_curve_fit STATIC ${START_DIR}/codegen/lib/step_z_curve_fit/rt_nonfinite.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/rtGetNaN.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/rtGetInf.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/step_z_curve_fit_initialize.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/step_z_curve_fit_terminate.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/step_z_curve_fit.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/curve_function.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/factoryConstruct.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/xnrm2.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/checkStoppingCriteria.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/hasFiniteBounds.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/computeFiniteDifferences.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/xgemv.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/projectBox.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/driver.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/linearLeastSquares.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/xzlarfg.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/xzlarf.cpp
    ${START_DIR}/codegen/lib/step_z_curve_fit/lsqcurvefit.cpp)
set_target_properties(step_z_curve_fit PROPERTIES
    PREFIX ""
    POSITION_INDEPENDENT_CODE ON
    RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/step_z_curve_fit/$<0:>"
    LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/step_z_curve_fit/$<0:>"
    ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/step_z_curve_fit/$<0:>")
target_compile_features(step_z_curve_fit PUBLIC cxx_std_11)
target_compile_definitions(step_z_curve_fit PRIVATE -DMODEL=step_z_curve_fit)
target_include_directories(step_z_curve_fit PUBLIC $<BUILD_INTERFACE:${START_DIR}/codegen/lib/step_z_curve_fit>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/codegen/lib/step_z_curve_fit>
    $<BUILD_INTERFACE:${START_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>)
target_link_libraries(step_z_curve_fit INTERFACE $<$<BOOL:${FOUND_LIBM}>:m>
    $<$<BOOL:${FOUND_LIBSTDCXX}>:stdc++>)


######################################################################
## Build success message
######################################################################
add_custom_command(TARGET step_z_curve_fit POST_BUILD
     COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "\\#\\#\\# Created library: $<TARGET_FILE:step_z_curve_fit>")

######################################################################
## Call toolchain hook function if defined
######################################################################
if(COMMAND toolchain_target_hook)
    toolchain_target_hook(step_z_curve_fit)
endif()
