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
    scripts/mujoco_model.py
    scripts/collect_mujoco_dynamics.py
    scripts/validate_mujoco_compensation.py
    scripts/mujoco_dynamics_pipeline.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    scripts/common.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch urdf results
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md requirements-mujoco.txt
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    neural_network/train_neural_dynamics.py
    neural_network/predict_neural_dynamics.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    neural_network/neural_dynamics_model.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    neural_network/training_config.json
    neural_network/README.md
  DESTINATION share/${PROJECT_NAME}/neural_network
)

ament_package()
