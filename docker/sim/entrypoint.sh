#!/bin/bash
set -e

# Copy files from mounted volume
if [ ! -d "~/PX4-Autopilot/Tools/simulation/gz/worlds/leo_p6.sdf" ]; then
    echo "Copying files from mounted volume..."
    cp -rf ~/ros2_ws/src/Leorover/leo_simulator-ros2/leo_gz_worlds/worlds/leo_p6.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds && \
    cp -rf ~/ros2_ws/src/Leorover/leo_simulator-ros2/leo_gz_worlds/models/Obstacles ~/PX4-Autopilot/Tools/simulation/gz/models && \
    rm -rf ~/PX4-Autopilot/Tools/simulation/gz/models/x500 && \
    cp -rf ~/ros2_ws/src/Leorover/drone_description/sdf/x500 ~/PX4-Autopilot/Tools/simulation/gz/models
fi

# Start the main process
exec "$@"



#RUN cp -rf ~/ros2_ws/src/Leorover/leo_simulator-ros2/leo_gz_worlds/worlds/leo_p6.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds && \
#    cp -rf ~/ros2_ws/src/Leorover/leo_simulator-ros2/leo_gz_worlds/models/Obstacles ~/PX4-Autopilot/Tools/simulation/gz/models && \
#    rm -rf ~/PX4-Autopilot/Tools/simulation/gz/models/x500 && \
#    cp -rf ~/ros2_ws/src/Leorover/drone_description/sdf/x500 ~/PX4-Autopilot/Tools/simulation/gz/models
