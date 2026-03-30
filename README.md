# P6
TEMP: remember these commands please :)

COLCON BUILD
cb

RUN OCTOMAP
ros2 run octomap_server octomap_server_node   --ros-args   -r cloud_in:=/cloud_registered_body   -p frame_id:=camera_init   -p base_frame_id:=body   -p resolution:=0.1   -p filter_speckles:=true   -p filter_ground_plane:=true   -p ground_filter.angle:=0.1   -p ground_filter.distance:=0.3   -p ground_filter.plane_distance:=0.1   -p sensor_model.max_range:=8.0   -p point_cloud_max_z:=1.2   -p point_cloud_min_z:=0.0


