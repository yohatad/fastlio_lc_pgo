# Point-LIO + PGO loop closure on the Pepper L2 rig -- the Point-LIO
# equivalent of fastlio_lc_l2.launch.py (same file, FAST-LIO backend). See
# that file for the detailed comments on why each piece is wired the way it
# is; this only documents what's DIFFERENT for the Point-LIO backend.
#
# pgo_node's cloud_topic and odom_topic are plain overridable ROS parameters
# (not hardcoded), and Point-LIO's registered-cloud topic name
# (/cloud_registered_body) is IDENTICAL to FAST-LIO's -- only odom_topic
# needs pointing at Point-LIO's /aft_mapped_to_init. No PGO/GTSAM source
# changes needed for this swap.
#
# save_directory defaults to a SEPARATE path from fastlio_lc_l2.launch.py's:
# PGO wipes its Scans/ subfolder on startup, so sharing a directory between
# the two backends would let one destroy the other's saved keyframes/map.
#
# Usage:
#   ros2 launch fastlio_lc_pgo pointlio_lc_l2.launch.py
#   ros2 bag play <bag> --clock --topics /points /imu/data /tf_static
#   (do NOT replay /tf -- see pepper_sensor_tf.launch.py's header)

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    point_lio_share = get_package_share_directory('point_lio')
    pgo_share = get_package_share_directory('fastlio_lc_pgo')
    sensor_tf_share = get_package_share_directory('pepper_slam')
    default_rviz_cfg = os.path.join(pgo_share, 'rviz', 'fastlio_lc.rviz')

    save_directory = LaunchConfiguration('save_directory')
    rviz = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    use_sim_time = LaunchConfiguration('use_sim_time')
    occupancy = LaunchConfiguration('occupancy')

    declare_save_directory_cmd = DeclareLaunchArgument(
        'save_directory', default_value='/home/yoha/Lidar/run_l2_lc_pointlio/pgo_output/',
        description='Directory where PGO writes optimized poses, odom poses, times and '
                    'keyframe scans (its Scans/ subfolder is wiped on startup). Kept '
                    'separate from fastlio_lc_l2.launch.py\'s default so the two '
                    'backends cannot wipe each other\'s saved runs.'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 with both the raw Point-LIO view and the loop-closure '
                    '(PGO) view pre-configured (/aft_pgo_map, /aft_pgo_path, '
                    '/loop_closure_constraints).'
    )
    declare_rviz_cfg_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_cfg,
        description='RViz config file path (shared with fastlio_lc_l2.launch.py -- '
                    'the PGO topics it visualizes are the same regardless of backend).'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the bag clock (ros2 bag play --clock). Keep true for offline runs.'
    )
    declare_occupancy_cmd = DeclareLaunchArgument(
        'occupancy', default_value='true',
        description='Run octomap_server on the per-scan /cloud_registered_body to '
                    'build a ray-traced 2D OccupancyGrid on /projected_map. Save it: '
                    'ros2 run nav2_map_server map_saver_cli -t /projected_map -f <name>'
    )
    # map is now FLOOR-referenced (level_frame_on_floor in
    # pgo_map_odom_bridge), so these are simply heights above the ground and
    # no longer carry a lidar-mount offset. Measured on the July_22 bag: floor
    # plane lands at z ~= -0.02 m, room ceiling ~ +3.7 m.
    declare_occ_min_z_cmd = DeclareLaunchArgument(
        'occ_min_z', default_value='0.20',
        description='occupancy_min_z for octomap: height above the FLOOR (map '
                    'z=0 is the ground plane). Must stay above the floor or the '
                    'traversed floor is marked occupied (a black trail wherever the '
                    'robot drives); 0.20 keeps the clearance the old lidar-referenced '
                    '-0.25 had, while still catching low obstacles.'
    )
    declare_occ_max_z_cmd = DeclareLaunchArgument(
        'occ_max_z', default_value='1.45',
        description='occupancy_max_z for octomap: height above the FLOOR. 1.45 m sits '
                    'just above Pepper, so nothing it cannot collide with is mapped.'
    )
    declare_self_hit_range_cmd = DeclareLaunchArgument(
        'self_hit_range', default_value='0.8',
        description='Drop scan points closer than this (m) before octomap, to remove '
                    'the low lidar seeing Pepper self-hits.'
    )
    declare_max_range_cmd = DeclareLaunchArgument(
        'max_range', default_value='20.0',
        description='octomap sensor_model.max_range (m).'
    )
    declare_ror_neighbors_cmd = DeclareLaunchArgument(
        'ror_min_neighbors', default_value='0',
        description='Radius-outlier removal: drop scan points with fewer than this many '
                    'neighbours within ror_radius. 0 = OFF (default).'
    )
    declare_ror_radius_cmd = DeclareLaunchArgument(
        'ror_radius', default_value='0.5',
        description='Radius (m) for radius-outlier removal neighbour count.'
    )
    declare_mapviz_filter_size_cmd = DeclareLaunchArgument(
        'mapviz_filter_size', default_value='0.1',
        description='Voxel leaf size (m) pgo_node downsamples /aft_pgo_map to before '
                    'publishing. Default in pgo_node itself is 0.4 (sparse/blocky); '
                    '0.1 gives a visibly denser accumulated map close to the raw '
                    'per-scan CloudRegistered look. Lower = denser but bigger/slower '
                    'to rebuild and republish each vizmapFrequency cycle.'
    )
    declare_map_save_filter_size_cmd = DeclareLaunchArgument(
        'map_save_filter_size', default_value='0.05',
        description='Voxel leaf size (m) for the map_batch.pcd written by '
                    '/pgo_batch_optimize -- the localization PRIOR that '
                    'lio_localization ICPs against, so denser is better. '
                    'Independent of mapviz_filter_size so a dense prior does '
                    'not also make the RViz map heavy. <=0 means "same as '
                    'mapviz_filter_size".'
    )
    occ_min_z = LaunchConfiguration('occ_min_z')
    occ_max_z = LaunchConfiguration('occ_max_z')
    self_hit_range = LaunchConfiguration('self_hit_range')
    max_range = LaunchConfiguration('max_range')
    ror_min_neighbors = LaunchConfiguration('ror_min_neighbors')
    ror_radius = LaunchConfiguration('ror_radius')
    mapviz_filter_size = LaunchConfiguration('mapviz_filter_size')
    map_save_filter_size = LaunchConfiguration('map_save_filter_size')

    # Static sensor rig: base_footprint -> l2lidar_frame -> l2lidar_frame_imu (+ cams).
    sensor_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensor_tf_share, 'launch', 'pepper_sensor_tf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Point-LIO owns odom_lidar -> base_footprint (via lio_map_odom_bridge,
    # same as FAST-LIO's variant). bridge_level_frame:='false' for the same
    # reason: PGO owns map_lidar -> odom_lidar below, so odom_lidar must keep a
    # single parent. The leveling happens one level up instead --
    # pgo_map_odom_bridge publishes map -> map_lidar -- so the leveled frame in
    # this stack is 'map', not 'odom'.
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_lio_share, 'launch', 'mapping_l2lidar_node.launch.py')
        ),
        launch_arguments={
            'rviz': rviz,
            'rviz_cfg': rviz_cfg,
            'use_sim_time': use_sim_time,
            'bridge_level_frame': 'false',
        }.items()
    )

    # PGO owns the loop-closure correction map -> odom, same as the FAST-LIO
    # variant -- odom_topic is the only thing that changes for Point-LIO.
    pgo_map_odom_bridge = Node(
        package='fast_lio',
        executable='pgo_map_odom_bridge.py',
        name='pgo_map_odom_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': 'map_lidar',
            'odom_frame': 'odom_lidar',
            'base_frame': 'base_footprint',
            'level_frame': 'map',
            'odom_topic': '/aft_mapped_to_init',
            'pgo_odom_topic': '/aft_pgo_odom',
            'publish_level_frame': True,
        }],
    )

    # NOTE: /cloud_registered_body must carry a frame_id that actually exists
    # in the TF tree, or octomap_server's tf2 MessageFilter drops every scan
    # and /projected_map stays empty forever. Point-LIO upstream hardcoded it
    # to the nonexistent "body"; it is now the publish.body_frame parameter,
    # set to l2lidar_frame_imu in point_lio/config/l2lidar_node.yaml (mirroring
    # FAST_LIO/config/l2.yaml). Nothing needed here.
    range_filter_node = Node(
        package='fast_lio',
        executable='cloud_range_filter.py',
        name='cloud_range_filter',
        output='screen',
        condition=IfCondition(occupancy),
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/cloud_registered_body',
            'output_topic': '/cloud_registered_body_filtered',
            'min_range': self_hit_range,
            'ror_min_neighbors': ror_min_neighbors,
            'ror_radius': ror_radius,
        }],
    )

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        condition=IfCondition(occupancy),
        remappings=[('cloud_in', '/cloud_registered_body_filtered')],
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'map',
            'base_frame_id': 'base_footprint',
            'resolution': 0.05,
            'sensor_model.max_range': max_range,
            'filter_speckles': True,
            'occupancy_min_z': occ_min_z,
            'occupancy_max_z': occ_max_z,
            'filter_ground_plane': True,
            'ground_filter.distance': 0.05,
            'ground_filter.angle': 0.15,
            'ground_filter.plane_distance': 0.12,
            'latch': True,
        }],
    )

    # No LD_LIBRARY_PATH override -- pgo_node is the identical executable
    # regardless of which LIO backend's topics it's pointed at, and a fresh
    # rebuild links it against ros-humble-gtsam 4.2.0 directly. See
    # fastlio_lc_l2.launch.py's comment at the equivalent spot for the full
    # story (a stale apt libgtsam4 4.1.1 this used to need is gone from this
    # system; forcing a different GTSAM here now BREAKS it instead).
    pgo_node = Node(
        package='fastlio_lc_pgo',
        executable='pgo_node',
        name='laserPGO',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'save_directory': save_directory,
            'cloud_topic': '/cloud_registered_body',  # Point-LIO's name matches FAST-LIO's
            'odom_topic': '/aft_mapped_to_init',       # only this differs from the FAST-LIO variant
            'map_frame': 'map_lidar',
            'keyframe_meter_gap': 1.0,
            'keyframe_deg_gap': 10.0,
            'sc_dist_thres': 0.4,
            'sc_max_radius': 20.0,
            'historyKeyframeSearchRadius': 1.5,
            'historyKeyframeSearchTimeDiff': 30.0,
            'historyKeyframeSearchNum': 20,
            'speedFactor': 1.0,
            'loopClosureFrequency': 4.0,
            'graphUpdateFrequency': 2.0,
            'graphUpdateTimes': 5,
            'loopNoiseScore': 0.1,
            'vizmapFrequency': 10.0,
            'loopFitnessScoreThreshold': 0.3,
            'mapviz_filter_size': mapviz_filter_size,
            'map_save_filter_size': map_save_filter_size,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_save_directory_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_cfg_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_occupancy_cmd)
    ld.add_action(declare_occ_min_z_cmd)
    ld.add_action(declare_occ_max_z_cmd)
    ld.add_action(declare_self_hit_range_cmd)
    ld.add_action(declare_max_range_cmd)
    ld.add_action(declare_ror_neighbors_cmd)
    ld.add_action(declare_ror_radius_cmd)
    ld.add_action(declare_mapviz_filter_size_cmd)
    ld.add_action(declare_map_save_filter_size_cmd)
    ld.add_action(sensor_tf_launch)
    ld.add_action(point_lio_launch)
    ld.add_action(pgo_node)
    ld.add_action(pgo_map_odom_bridge)
    ld.add_action(range_filter_node)
    ld.add_action(octomap_node)
    return ld
