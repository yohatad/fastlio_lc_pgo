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
#   (replaying /tf is SAFE and wanted -- see pepper_sensor_tf.launch.py's
#    header for why the old "do not replay /tf" advice no longer holds.)

import os
from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        'save_directory',
        default_value=os.path.expanduser('~/Lidar/run_l2_lc_pointlio/pgo_output/'),
        description='Directory where PGO writes optimized poses, odom poses, times and '
                    'keyframe scans (its Scans/ subfolder is wiped on startup). Kept '
                    'separate from fastlio_lc_l2.launch.py\'s default so the two '
                    'backends cannot wipe each other\'s saved runs.'
    )

    # The finished 3D map goes into pepper_navigation/pcd, beside the keyframe
    # poses it is paired with -- not into save_directory, which is scratch (this
    # node wipes <save_directory>/Scans at startup and fills it with one .pcd
    # per keyframe plus pose logs).
    # Written to the SOURCE tree, not the install share, so it survives a
    # rebuild -- pepper_navigation's CMakeLists installs pcd/*.pcd from there.
    # os.path.expanduser, not a literal /home/yoha: the old default only ever
    # resolved on one machine, which is the anti-pattern the bag wrappers and
    # pepper_navigation/CMakeLists.txt were both cleaned of. The remaining
    # assumption is that the workspace sits at ~/ros2_ws; pass the argument if
    # it does not. Empty falls back to <save_directory>/map_batch.pcd.
    declare_map_pcd_path_cmd = DeclareLaunchArgument(
        'map_pcd_path',
        default_value=os.path.expanduser(
            '~/ros2_ws/src/pepper4dec/pepper_navigation/pcd/pepper_map_lc.pcd'),
        description='Full path of the map written by /pgo_batch_optimize. Empty '
                    'falls back to <save_directory>/map_batch.pcd.'
    )

    # MUST be declared here, not only in the bag wrapper: ROS 2 launch silently
    # DROPS launch_arguments an included description does not declare. The
    # wrapper passed keyframe_filter_size:=0.25 for months and pgo_node kept its
    # own 0.4 default (verified with `ros2 param get /laserPGO
    # keyframe_filter_size`), so every map was coarser than intended. This leaf
    # is applied BEFORE a keyframe is stored, so map_save_filter_size can never
    # recover the resolution it discarded.
    declare_keyframe_filter_size_cmd = DeclareLaunchArgument(
        'keyframe_filter_size', default_value='0.25',
        description='Voxel leaf (m) applied to each keyframe BEFORE storage, so '
                    'it bounds the density of every downstream product. 0.25 '
                    "matches FAST-LIO's own filter_size_surf, the real floor."
    )
    # Down a corridor the two long walls give the scan matcher no vertical
    # constraint, so keyframe height drifts and loop closure cannot pull it
    # back. Pins every keyframe to the first one's height and nothing else.
    # Measured on slam_20260823_merged (FAST-LIO, same rig): trajectory
    # vertical spread 5.14 m -> 0.09 m, horizontal extent unchanged.
    declare_planar_prior_cmd = DeclareLaunchArgument(
        'planar_prior', default_value='true',
        description='Constrain keyframe height to the floor plane. Turn off '
                    'only if the robot actually changes level (ramp, lift).')
    # Derived at runtime from the same map <- pgo_init transform the saved
    # cloud is leveled by, so the prior cannot hold a different "up" than the
    # map does. That also makes this correct for Point-LIO without re-measuring
    # anything: its world frame is its own, and the TF describes it.
    declare_planar_gravity_auto_cmd = DeclareLaunchArgument(
        'planar_gravity_auto', default_value='true',
        description='Derive the prior axis from the leveling TF. Set false only '
                    'to pin it by hand via planar_gravity.')
    declare_planar_gravity_cmd = DeclareLaunchArgument(
        'planar_gravity', default_value='[-0.0075, 1.0, 0.0031]',
        description='Unit gravity in the LIO world frame; ignored unless '
                    'planar_gravity_auto is false.')
    declare_planar_sigma_cmd = DeclareLaunchArgument(
        'planar_sigma_h', default_value='0.05',
        description='Std dev [m] of how far off the floor plane a keyframe may '
                    'sit. Loosen if the floor is genuinely uneven.')

    declare_lio_config_file_cmd = DeclareLaunchArgument(
        'lio_config_file', default_value='l2lidar_rsimu.yaml',
        description='Point-LIO config. l2lidar_rsimu.yaml uses the RealSense '
                    'IMU (default); l2lidar_node.yaml uses the L2 s own -- see '
                    'utils/L2_IMU/REPORT.md.')
    declare_lidar_imu_frame_cmd = DeclareLaunchArgument(
        'lidar_imu_frame', default_value='camera_imu_optical_frame',
        description='Static frame matching the config s publish.body_frame. '
                    'camera_imu_optical_frame for l2lidar_rsimu.yaml, '
                    'l2lidar_frame_imu for l2lidar_node.yaml.')
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 with both the raw Point-LIO view and the loop-closure '
                    '(PGO) view pre-configured (/aft_pgo_map, /aft_pgo_path, '
                    '/loop_closure_constraints).'
    )
    declare_rviz_cfg_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_cfg,
        description='RViz config file path (shared with fastlio_lc_l2.launch.py -- '
                    'the PGO topics it visualizes are the same regardless of backend).'
    )
    # false, NOT true: this is the LIVE entry point. Every wrapper in
    # pepper_slam/launch/bag_test sets use_sim_time:='true' explicitly, so this
    # default only ever applies on the robot -- where 'true' pins sim time at 0,
    # so tf never resolves and nothing fuses, silently and with no error.
    # pepper_sensor_tf's 'publisher'/'scope' are NOT derived from this -- only
    # use_sim_time is forwarded. On a bag, pass them yourself: publisher:=none
    # if it carries its own /tf_static, publisher:=urdf scope:=all if it does
    # not. The bag_test wrappers already default publisher to none.
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='false (default) on the robot; true for bag replay with ros2 bag play --clock. The bag_test wrappers set this for you.'
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

    # Point-LIO owns lio_init -> base_footprint (via lio_odom_bridge,
    # same as FAST-LIO's variant). bridge_level_frame:='false' for the same
    # reason: PGO owns pgo_init -> lio_init below, so lio_init must keep a
    # single parent. The leveling happens one level up instead --
    # pgo_map_odom_bridge publishes map -> pgo_init -- so the leveled frame in
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
            'config_file': LaunchConfiguration('lio_config_file'),
        }.items()
    )

    # PGO owns the loop-closure correction map -> odom, same as the FAST-LIO
    # variant -- odom_topic is the only thing that changes for Point-LIO.
    pgo_map_odom_bridge = Node(
        package='pepper_slam',
        executable='pgo_map_odom_bridge.py',
        name='pgo_map_odom_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': 'pgo_init',
            'odom_frame': 'lio_init',
            'base_frame': 'base_footprint',
            'level_frame': 'map',
            'odom_topic': '/odom_lio',
            'pgo_odom_topic': '/aft_pgo_odom',
            'publish_level_frame': True,
            # MUST match the LIO config's publish.body_frame. level_source
            # 'calibration' builds the map -> pgo_init levelling rotation from
            # base_frame -> lidar_imu_frame; left at the node's
            # l2lidar_frame_imu default while Point-LIO estimates the RealSense
            # IMU, the whole map is levelled by the WRONG mount and comes out
            # roughly 90 deg off, with no error logged anywhere.
            'lidar_imu_frame': LaunchConfiguration('lidar_imu_frame'),
        }],
    )

    # NOTE: /cloud_registered_body must carry a frame_id that actually exists
    # in the TF tree, or octomap_server's tf2 MessageFilter drops every scan
    # and /projected_map stays empty forever. Point-LIO upstream hardcoded it
    # to the nonexistent "body"; it is now the publish.body_frame parameter,
    # set to l2lidar_frame_imu in point_lio/config/l2lidar_node.yaml (mirroring
    # FAST_LIO/config/l2.yaml). Nothing needed here.
    range_filter_node = Node(
        package='pepper_slam',
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
            'map_pcd_path': LaunchConfiguration('map_pcd_path'),
            # MUST match pgo_map_odom_bridge's level_frame below ('map').
            # pgo_node's own default is 'map_level', and when the two disagree
            # its canTransform(level_frame, map_frame) fails, so it WARNS and
            # silently saves the .pcd in the RAW pgo_init frame instead --
            # ~90 deg off gravity (measured -92.2 deg roll / -88.1 deg yaw on
            # 2026-08-22), which does not match the 2D grid octomap builds in
            # 'map'. The service still returns success=True, so the only clue
            # is the "frame pgo_init" in its message. Recover a map already
            # saved that way with utils/build_pgo_map.py --level-tf.
            'level_frame': 'map',
            'keyframe_filter_size': LaunchConfiguration('keyframe_filter_size'),
            'cloud_topic': '/cloud_registered_body',  # Point-LIO's name matches FAST-LIO's
            'odom_topic': '/odom_lio',       # only this differs from the FAST-LIO variant
            'map_frame': 'pgo_init',

            # planar-motion prior -- see declare_planar_prior_cmd above
            'planar_prior_enable': ParameterValue(
                LaunchConfiguration('planar_prior'), value_type=bool),
            'planar_gravity_auto': ParameterValue(
                LaunchConfiguration('planar_gravity_auto'), value_type=bool),
            'planar_gravity': ParameterValue(
                LaunchConfiguration('planar_gravity'), value_type=List[float]),
            'planar_sigma_h': ParameterValue(
                LaunchConfiguration('planar_sigma_h'), value_type=float),

            'keyframe_meter_gap': 1.0,
            'keyframe_deg_gap': 10.0,
            # LOOP-CLOSURE ACCEPTANCE GATES, tightened 2026-08-10 after a run on
            # bag/slam_august_8_bag folded the map. The three values below had
            # drifted apart from the pgo_node defaults in the permissive
            # direction ALL AT ONCE, which is what let bad closures both get in
            # and dominate:
            #   sc_dist_thres              0.4 vs default 0.2  (2x looser detector)
            #   loopFitnessScoreThreshold  0.3 vs default 0.3  (loose ICP gate)
            #   loopNoiseScore             0.1 vs default 0.5  (5x MORE trusted)
            # Observed in that run: closures accepted at ICP fitness 0.151, 0.178
            # and 0.081 alongside rejects at 0.359/0.488 -- i.e. sitting right on
            # the gate -- with query keyframes repeatedly matching one old node
            # (89 -> 379/380/381, 220 -> 449..457). Indoor corridors are
            # self-similar and the L2 gives only ~5.3k points per scan, so Scan
            # Context descriptors are weak here and false positives are expected;
            # the gates, not the detector, are what must be strict.
            #
            # There IS a Cauchy robust kernel on loop factors
            # (laserPosegraphOptimization.cpp:303-305), but at variance 0.1 it
            # cannot absorb a wrong constraint -- hence raising loopNoiseScore too.
            'sc_dist_thres': 0.2,   # was 0.4 -- back to the pgo_node default
            'sc_max_radius': 20.0,
            'historyKeyframeSearchRadius': 1.5,
            'historyKeyframeSearchTimeDiff': 30.0,
            'historyKeyframeSearchNum': 20,
            'speedFactor': 1.0,
            'loopClosureFrequency': 4.0,
            'graphUpdateFrequency': 2.0,
            'graphUpdateTimes': 5,
            # was 0.1: GTSAM variance on loop factors. LOWER = trusted MORE, so
            # 0.1 made every accepted closure pull 5x harder than the pgo_node
            # default (0.5). 0.3 still favours closures over odometry without
            # letting a single bad one yank the graph. See the gate note above.
            'loopNoiseScore': 0.3,
            'vizmapFrequency': 10.0,
            # was 0.3: ICP fitness gate. 0.15 rejects the marginal 0.151/0.178
            # passes seen in the folded run while keeping the clean 0.02-0.08 ones.
            'loopFitnessScoreThreshold': 0.15,
            'mapviz_filter_size': mapviz_filter_size,
            'map_save_filter_size': map_save_filter_size,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_save_directory_cmd)
    ld.add_action(declare_map_pcd_path_cmd)
    ld.add_action(declare_keyframe_filter_size_cmd)
    ld.add_action(declare_planar_prior_cmd)
    ld.add_action(declare_planar_gravity_auto_cmd)
    ld.add_action(declare_planar_gravity_cmd)
    ld.add_action(declare_planar_sigma_cmd)
    ld.add_action(declare_lio_config_file_cmd)
    ld.add_action(declare_lidar_imu_frame_cmd)
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
