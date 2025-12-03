#! /bin/bash
bazel run -c opt --copt="-D__ASPICE_TIMEING_TEST__=1" onboard/planner/plan/acc/aspice_timing_test:acc_preliminary_speed_timing > acc_preliminary_speed_timing_log.txt
dateline1=$(cat onboard/planner/plan/acc/aspice_timing_test/acc_preliminary_speed_time_base.log)
dateline2=$(cat acc_preliminary_speed_timing_log.txt)
if [ "$dateline1" = "$dateline2" ]; then
  echo -e "\033[37mACC_PRELIMINARY_SPEED_TIMING\033[0m \033[32mOK\033[0m"
else
  echo -e "\033[37mACC_PRELIMINARY_SPEED_TIMING\033[0m \033[31mFAILED\033[0m"
fi
