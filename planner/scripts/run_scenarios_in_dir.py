# Example usage:
# onboard/planner/scripts/run_scenarios_in_dir.py /qcraft/onboard/planner/testdata/dojo_scenarios 2> /tmp/log.txt
# cat /tmp/log.txt | grep -E "((Metric \w* FAILED)|(scenario_path))"
import os
import sys

scenarios = os.listdir(sys.argv[1])
for scenario in scenarios:
    cmd = "bazel run -c opt offboard/simulation:simulator_main -- --scenario_path='{}' --sim_mode=dsim".format(
        sys.argv[1] + "/" + scenario
    )
    print(cmd)
    os.system(cmd)
