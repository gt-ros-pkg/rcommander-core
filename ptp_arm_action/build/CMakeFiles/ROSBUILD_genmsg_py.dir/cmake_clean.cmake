FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ptp_arm_action/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ptp_arm_action/msg/__init__.py"
  "../src/ptp_arm_action/msg/_LinearMovementAction.py"
  "../src/ptp_arm_action/msg/_LinearMovementGoal.py"
  "../src/ptp_arm_action/msg/_LinearMovementActionGoal.py"
  "../src/ptp_arm_action/msg/_LinearMovementResult.py"
  "../src/ptp_arm_action/msg/_LinearMovementActionResult.py"
  "../src/ptp_arm_action/msg/_LinearMovementFeedback.py"
  "../src/ptp_arm_action/msg/_LinearMovementActionFeedback.py"
  "../msg/LinearMovementAction.msg"
  "../msg/LinearMovementGoal.msg"
  "../msg/LinearMovementActionGoal.msg"
  "../msg/LinearMovementResult.msg"
  "../msg/LinearMovementActionResult.msg"
  "../msg/LinearMovementFeedback.msg"
  "../msg/LinearMovementActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
