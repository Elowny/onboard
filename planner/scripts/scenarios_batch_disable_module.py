import os
import sys

# Set spin_publisher_mdoule and image_publisher_module to "disable: true"
for f in os.listdir("onboard/planner/testdata/scenarios/"):
    if f[-4:] != ".txt":
        continue
    print(f)
    cmd = "sed '/spin_publisher_module/!b;n;n;c\      disable: true' onboard/planner/testdata/scenarios/{0} > /tmp/tmp.txt".format(
        f
    )
    os.system(cmd)
    cmd = "cp /tmp/tmp.txt onboard/planner/testdata/scenarios/{0}".format(f)
    os.system(cmd)
    cmd = "sed '/image_publisher_module/!b;n;n;c\      disable: true' onboard/planner/testdata/scenarios/{0} > /tmp/tmp.txt".format(
        f
    )
    os.system(cmd)
    cmd = "cp /tmp/tmp.txt onboard/planner/testdata/scenarios/{0}".format(f)
    os.system(cmd)
