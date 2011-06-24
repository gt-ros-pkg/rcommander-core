FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ptp_arm_action/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
