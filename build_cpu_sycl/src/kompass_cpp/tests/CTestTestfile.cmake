# CMake generated Testfile for 
# Source directory: /workspace/kompass-core/src/kompass_cpp/tests
# Build directory: /workspace/kompass-core/build_cpu_sycl/src/kompass_cpp/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(controller_tests "controller_test")
set_tests_properties(controller_tests PROPERTIES  _BACKTRACE_TRIPLES "/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;20;add_test;/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;0;")
add_test(trajectory_sampler_tests "trajectory_sampler_test")
set_tests_properties(trajectory_sampler_tests PROPERTIES  _BACKTRACE_TRIPLES "/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;25;add_test;/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;0;")
add_test(cost_evaluator_tests "cost_evaluator_test")
set_tests_properties(cost_evaluator_tests PROPERTIES  _BACKTRACE_TRIPLES "/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;30;add_test;/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;0;")
add_test(mapper_tests "pointcloud_test")
set_tests_properties(mapper_tests PROPERTIES  _BACKTRACE_TRIPLES "/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;35;add_test;/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;0;")
add_test(collisions_tests "collisions_test")
set_tests_properties(collisions_tests PROPERTIES  _BACKTRACE_TRIPLES "/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;45;add_test;/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;0;")
add_test(critical_zone_tests "critical_zone_test")
set_tests_properties(critical_zone_tests PROPERTIES  _BACKTRACE_TRIPLES "/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;50;add_test;/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;0;")
add_test(vision_tests "vision_tracking_test")
set_tests_properties(vision_tests PROPERTIES  _BACKTRACE_TRIPLES "/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;55;add_test;/workspace/kompass-core/src/kompass_cpp/tests/CMakeLists.txt;0;")
