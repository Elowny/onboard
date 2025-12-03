#include "onboard/planner/ml/optimizer_auto_tuning/auto_tuning_common_flags.h"

DEFINE_bool(dump_expert_policy, false,
            "Whether to output the expert policy(expert knowledge).");
DEFINE_string(specific_snapshot_folder,
              "/hosthome/DAT_data/training_set/planner_state_proto_2999/",
              "Only used in auto tuning mode, it is the folder name of a "
              "specific snapshot. "
              "Make sure the folder exists, see the default value as example.");
DEFINE_string(auto_tuning_data_filename, "auto_tuning_data.pb.txt",
              "The file name of the auto tuning data used to interact between "
              "c++ and python.");
DEFINE_bool(output_pose_traj, false,
            "Whether to dump pose traj as TrajectoryProto based on the current "
            "use cost weight.");
