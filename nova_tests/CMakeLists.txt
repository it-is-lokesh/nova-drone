cmake_minimum_required(VERSION 3.8)
project(nova_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nova_processing REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../nova_processing/include
)

add_executable(log_imu src/log_imu.cpp)

add_executable(mat_mul src/mat_mul.cpp)

install(TARGETS
  log_imu
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  mat_mul
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(log_imu PUBLIC nova_processing::nova_processing)
target_link_libraries(mat_mul PUBLIC nova_processing::nova_processing)


add_executable(shm_manager src/shm_manager.cpp)
add_executable(producer src/producer.cpp)
add_executable(consumer src/consumer.cpp)

install(TARGETS
  shm_manager
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  producer
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  consumer
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(shm_manager PUBLIC nova_processing::nova_processing)
target_link_libraries(producer PUBLIC nova_processing::nova_processing)
target_link_libraries(consumer PUBLIC nova_processing::nova_processing)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
