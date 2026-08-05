cmake_minimum_required(VERSION 3.8)
project(exo_identify)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/design_excitation.py
    scripts/identify_parameters.py
    scripts/export_gravity_formula.py
    scripts/gravity_compensation.py
    scripts/trajectory_experiment.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    scripts/common.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
