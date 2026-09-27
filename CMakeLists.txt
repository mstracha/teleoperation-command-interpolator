cmake_minimum_required(VERSION 3.20)

project(
    TeleoperationCommandInterpolator
    VERSION 1.0.0
    LANGUAGES CXX
)

include(CTest)
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

find_package(Threads REQUIRED)

option(
    TELEOPERATION_BUILD_TOOLS
    "Build command-line diagnostic and qualification tools"
    ON
)

set(TELEOPERATION_PUBLIC_HEADERS
    include/teleoperation/CommandDerivativeEstimator.hpp
    include/teleoperation/CommandInterpolator.hpp
    include/teleoperation/CommandRingBuffer.hpp
    include/teleoperation/CommandTypes.hpp
    include/teleoperation/FaultReason.hpp
    include/teleoperation/InterpolatorConfiguration.hpp
    include/teleoperation/InterpolatorResults.hpp
    include/teleoperation/InterpolatorStateMachine.hpp
    include/teleoperation/JointLimits.hpp
    include/teleoperation/JointTrajectorySegment.hpp
    include/teleoperation/QuinticSegment.hpp
    include/teleoperation/QuinticValidator.hpp
    include/teleoperation/RealTimeHarness.hpp
    include/teleoperation/RunReportWriter.hpp
    include/teleoperation/ShutdownCoordinator.hpp
    include/teleoperation/TelemetryRingBuffer.hpp
    include/teleoperation/TelemetrySamples.hpp
    include/teleoperation/VirtualTimeHarness.hpp
)

add_library(teleoperation_command_interpolator STATIC
    ${TELEOPERATION_PUBLIC_HEADERS}
    src/CommandDerivativeEstimator.cpp
    src/CommandInterpolator.cpp
    src/InterpolatorStateMachine.cpp
    src/JointTrajectorySegment.cpp
    src/QuinticSegment.cpp
    src/QuinticValidator.cpp
    src/RealTimeHarness.cpp
    src/RunReportWriter.cpp
    src/ShutdownCoordinator.cpp
    src/VirtualTimeHarness.cpp
)

add_library(
    teleoperation::command_interpolator
    ALIAS teleoperation_command_interpolator
)

target_compile_features(
    teleoperation_command_interpolator
    PUBLIC cxx_std_17
)

target_include_directories(teleoperation_command_interpolator
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(
    teleoperation_command_interpolator
    PUBLIC Threads::Threads
)

set_target_properties(teleoperation_command_interpolator PROPERTIES
    CXX_EXTENSIONS OFF
    EXPORT_NAME command_interpolator
)

function(teleoperation_enable_warnings target_name)
    if(MSVC)
        target_compile_options(${target_name} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(
            ${target_name}
            PRIVATE -Wall -Wextra -Wpedantic
        )
    endif()
endfunction()

teleoperation_enable_warnings(teleoperation_command_interpolator)

if(TELEOPERATION_BUILD_TOOLS)
    add_executable(
        timing_qualification
        tools/TimingQualification.cpp
    )
    target_link_libraries(
        timing_qualification
        PRIVATE teleoperation::command_interpolator
    )
    target_compile_features(timing_qualification PRIVATE cxx_std_17)
    set_target_properties(timing_qualification PROPERTIES CXX_EXTENSIONS OFF)
    teleoperation_enable_warnings(timing_qualification)
endif()

if(BUILD_TESTING)
    function(teleoperation_add_test target_name test_name source_file)
        add_executable(${target_name} ${source_file})
        target_link_libraries(
            ${target_name}
            PRIVATE teleoperation::command_interpolator
        )
        target_compile_features(${target_name} PRIVATE cxx_std_17)
        set_target_properties(${target_name} PROPERTIES CXX_EXTENSIONS OFF)
        teleoperation_enable_warnings(${target_name})
        add_test(NAME ${test_name} COMMAND ${target_name})
    endfunction()

    teleoperation_add_test(
        interpolator_state_machine_tests
        InterpolatorStateMachineTests
        tests/InterpolatorStateMachineTests.cpp
    )
    teleoperation_add_test(
        command_interpolator_tests
        CommandInterpolatorTests
        tests/CommandInterpolatorTests.cpp
    )
    teleoperation_add_test(
        shutdown_coordinator_tests
        ShutdownCoordinatorTests
        tests/ShutdownCoordinatorTests.cpp
    )
    teleoperation_add_test(
        quintic_segment_tests
        QuinticSegmentTests
        tests/QuinticSegmentTests.cpp
    )
    teleoperation_add_test(
        joint_trajectory_segment_tests
        JointTrajectorySegmentTests
        tests/JointTrajectorySegmentTests.cpp
    )
    teleoperation_add_test(
        command_derivative_estimator_tests
        CommandDerivativeEstimatorTests
        tests/CommandDerivativeEstimatorTests.cpp
    )
    teleoperation_add_test(
        quintic_validator_tests
        QuinticValidatorTests
        tests/QuinticValidatorTests.cpp
    )
    teleoperation_add_test(
        virtual_time_harness_tests
        VirtualTimeHarnessTests
        tests/VirtualTimeHarnessTests.cpp
    )
    teleoperation_add_test(
        real_time_harness_tests
        RealTimeHarnessTests
        tests/RealTimeHarnessTests.cpp
    )
    teleoperation_add_test(
        run_report_writer_tests
        RunReportWriterTests
        tests/RunReportWriterTests.cpp
    )
endif()

install(
    TARGETS teleoperation_command_interpolator
    EXPORT TeleoperationCommandInterpolatorTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

configure_package_config_file(
    cmake/TeleoperationCommandInterpolatorConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/TeleoperationCommandInterpolatorConfig.cmake
    INSTALL_DESTINATION
        ${CMAKE_INSTALL_LIBDIR}/cmake/TeleoperationCommandInterpolator
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/TeleoperationCommandInterpolatorConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    EXPORT TeleoperationCommandInterpolatorTargets
    FILE TeleoperationCommandInterpolatorTargets.cmake
    NAMESPACE teleoperation::
    DESTINATION
        ${CMAKE_INSTALL_LIBDIR}/cmake/TeleoperationCommandInterpolator
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/TeleoperationCommandInterpolatorConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/TeleoperationCommandInterpolatorConfigVersion.cmake
    DESTINATION
        ${CMAKE_INSTALL_LIBDIR}/cmake/TeleoperationCommandInterpolator
)
