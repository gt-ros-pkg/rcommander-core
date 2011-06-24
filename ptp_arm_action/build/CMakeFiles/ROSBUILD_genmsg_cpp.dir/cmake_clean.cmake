FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ptp_arm_action/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/ptp_arm_action/LinearMovementAction.h"
  "../msg_gen/cpp/include/ptp_arm_action/LinearMovementGoal.h"
  "../msg_gen/cpp/include/ptp_arm_action/LinearMovementActionGoal.h"
  "../msg_gen/cpp/include/ptp_arm_action/LinearMovementResult.h"
  "../msg_gen/cpp/include/ptp_arm_action/LinearMovementActionResult.h"
  "../msg_gen/cpp/include/ptp_arm_action/LinearMovementFeedback.h"
  "../msg_gen/cpp/include/ptp_arm_action/LinearMovementActionFeedback.h"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
