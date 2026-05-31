find_package(nlohmann_json 3.2.0 REQUIRED)

# Toggles power monitoring
option(ENABLE_POWER_MONITOR "Enable power monitoring (may affect benchmark accuracy)" OFF)

# Build-mode toggles:
#   BENCHMARK_WITH_NAV2    ON  -> build + run the Nav2 comparison workloads
#                                 (needs ROS 2 + Nav2). Turn OFF for the GPU
#                                 device builds (CUDA / ROCm) that only need the
#                                 kompass kernels and have no ROS install.
#   BENCHMARK_WITH_KOMPASS ON  -> time + report the kompass-core workloads.
#                                 Turn OFF for the Nav2-reference container,
#                                 which emits a JSON of only the Nav2 numbers
#                                 (a peer of the device JSONs).
option(BENCHMARK_WITH_NAV2 "Build the Nav2 comparison workloads" ON)
option(BENCHMARK_WITH_KOMPASS "Time and report the kompass-core workloads" ON)

if(NOT BENCHMARK_WITH_NAV2 AND NOT BENCHMARK_WITH_KOMPASS)
    message(FATAL_ERROR
        "At least one of BENCHMARK_WITH_NAV2 / BENCHMARK_WITH_KOMPASS must be ON")
endif()

# Define the benchmark executable
add_executable(kompass_benchmark
    benchmark_runner.cpp
)

# Apply Feature Flags
if(ENABLE_POWER_MONITOR)
    message(STATUS "Benchmark Power Monitoring: ENABLED (Threaded polling active)")
    target_compile_definitions(kompass_benchmark PRIVATE ENABLE_POWER_MONITOR)
else()
    message(STATUS "Benchmark Power Monitoring: DISABLED (Pure performance mode)")
endif()

# Build-mode compile definitions (consumed by benchmark_runner.cpp)
if(BENCHMARK_WITH_NAV2)
    target_compile_definitions(kompass_benchmark PRIVATE BENCH_NAV2=1)
else()
    target_compile_definitions(kompass_benchmark PRIVATE BENCH_NAV2=0)
endif()
if(BENCHMARK_WITH_KOMPASS)
    target_compile_definitions(kompass_benchmark PRIVATE BENCH_KOMPASS=1)
else()
    target_compile_definitions(kompass_benchmark PRIVATE BENCH_KOMPASS=0)
endif()
message(STATUS "Benchmark workloads: Nav2=${BENCHMARK_WITH_NAV2}  kompass=${BENCHMARK_WITH_KOMPASS}")

# Standard Include Directories
target_include_directories(kompass_benchmark PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/kompass_cpp/include
    ${PROJECT_SOURCE_DIR}/kompass_cpp/tests
)

# Link against the main library + core deps (always present)
target_link_libraries(kompass_benchmark PRIVATE
    kompass
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
)

# Nav2 comparison dependencies (only when the Nav2 workloads are built)
if(BENCHMARK_WITH_NAV2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(nav2_mppi_controller REQUIRED)
    find_package(nav2_costmap_2d REQUIRED)
    find_package(nav2_core REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav2_msgs REQUIRED)
    find_package(PCL REQUIRED)
    find_package(nav2_collision_monitor REQUIRED)

    target_include_directories(kompass_benchmark PRIVATE
        ${rclcpp_INCLUDE_DIRS}
        ${nav2_mppi_controller_INCLUDE_DIRS}
        ${nav2_costmap_2d_INCLUDE_DIRS}
        ${nav2_core_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${nav2_msgs_INCLUDE_DIRS}
        ${nav2_collision_monitor_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
    )

    target_link_libraries(kompass_benchmark PRIVATE
        ${rclcpp_LIBRARIES}
        ${nav2_mppi_controller_LIBRARIES}
        ${nav2_costmap_2d_LIBRARIES}
        ${nav2_core_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${nav2_msgs_LIBRARIES}
        ${nav2_collision_monitor_LIBRARIES}
        ${PCL_LIBRARIES}
    )
endif()

# Handle AdaptiveCpp
if(AdaptiveCpp_FOUND)
    target_compile_definitions(kompass_benchmark PRIVATE GPU=1)
    add_sycl_to_target(TARGET kompass_benchmark SOURCES benchmark_runner.cpp)
endif()
