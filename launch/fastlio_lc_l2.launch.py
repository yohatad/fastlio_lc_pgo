# FAST-LIO + pose-graph loop closure (MAPPING) on the Unitree L2.
#
#   pepper_sensor_tf     static rig TF
#   FAST-LIO             odometry -> /odom_lio, /cloud_registered_body
#   lio_odom_bridge      lio_init -> base_footprint
#   pgo_node             loop closure -> /aft_pgo_odom, /aft_pgo_map
#   pgo_map_odom_bridge  map -> pgo_init -> lio_init, and the upright 'map'
#   octomap_server       ray-traced 2D grid on /projected_map (occupancy:=true)
#
# Nothing is written until you call
#   ros2 service call /pgo_batch_optimize std_srvs/srv/Trigger
# which saves map_pcd_path plus optimized_poses.txt under save_directory.

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
    fast_lio_share = get_package_share_directory('fast_lio')
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
        default_value=os.path.expanduser('~/Lidar/run_l2_lc/pgo_output/'),
        description='Directory where PGO writes optimized poses, odom poses, times and keyframe scans (its Scans/ subfolder is wiped on startup)'
    )

    # Kept out of save_directory, which is scratch: this node wipes
    # <save_directory>/Scans at startup. Writes to the SOURCE tree so the map
    # survives a rebuild and pepper_navigation installs it from pcd/. Assumes
    # the workspace is at ~/ros2_ws; pass the argument if it is not.
    declare_map_pcd_path_cmd = DeclareLaunchArgument(
        'map_pcd_path',
        default_value=os.path.expanduser(
            '~/ros2_ws/src/pepper4dec/pepper_navigation/pcd/pepper_map_lc.pcd'),
        description='Full path of the map written by /pgo_batch_optimize. Empty '
                    'falls back to <save_directory>/map_batch.pcd.'
    )

    # MUST be declared here, not only in the bag wrapper: launch silently DROPS
    # launch_arguments an included description does not declare, and this one
    # went unnoticed for months. Applied BEFORE storage, so map_save_filter_size
    # can never recover the resolution it discards.
    declare_keyframe_filter_size_cmd = DeclareLaunchArgument(
        'keyframe_filter_size', default_value='0.25',
        description='Voxel leaf (m) applied to each keyframe BEFORE storage, so '
                    'it bounds the density of every downstream product. 0.25 '
                    "matches FAST-LIO's own filter_size_surf, the real floor."
    )
    # Down a corridor the two long walls give the scan matcher nothing to fix
    # height against, so z drifts without bound and loop closure cannot pull it
    # back: measured on slam_20260823_merged, an 80.3 m z band that a full batch
    # re-optimization over 2736 keyframes moved only 1.2%. This pins every
    # keyframe to the first one's height and nothing else.
    declare_planar_prior_cmd = DeclareLaunchArgument(
        'planar_prior', default_value='true',
        description='Constrain keyframe height to the floor plane. Turn off '
                    'only if the robot actually changes level (ramp, lift).')
    # Which axis the prior holds, derived from the same transform the saved
    # cloud is leveled by so the two cannot disagree. The hand-set value below
    # was 2.43 deg off, which leaked 3.0 m of height back over an 85 m corridor.
    declare_planar_gravity_auto_cmd = DeclareLaunchArgument(
        'planar_gravity_auto', default_value='true',
        description='Derive the prior axis from the leveling TF. Set false only '
                    'to pin it by hand via planar_gravity.')
    # Fallback only, used when planar_gravity_auto is false. In the pose graph's
    # own frame -- the IMU mount orientation at t=0, NOT gravity-aligned -- so
    # for the RealSense IMU this reads along +Y, emphatically not [0, 0, 1].
    declare_planar_gravity_cmd = DeclareLaunchArgument(
        'planar_gravity', default_value='[-0.0075, 1.0, 0.0031]',
        description='Unit gravity in the LIO world frame; ignored unless '
                    'planar_gravity_auto is false.')
    declare_planar_sigma_cmd = DeclareLaunchArgument(
        'planar_sigma_h', default_value='0.05',
        description='Std dev [m] of how far off the floor plane a keyframe may '
                    'sit. Loosen if the floor is genuinely uneven.')

    # The L2's own gyro cancels rotation about the gravity axis below ~16 deg/s
    # and cost 139 deg of heading over a 744 s run (utils/L2_IMU/REPORT.md) --
    # asking loop closure to repair a known systematic yaw error. The RealSense
    # measured 3.8% -> 2.4% mean yaw error, 11.2% -> 4.6% worst.
    declare_lio_config_file_cmd = DeclareLaunchArgument(
        'lio_config_file', default_value='l2_rsimu.yaml',
        description='FAST-LIO config. l2_rsimu.yaml uses the RealSense IMU '
                    '(recommended); l2.yaml uses the L2 s own.'
    )
    # Must match the IMU the config selects, or lio_odom_bridge closes
    # odom -> base_footprint through the wrong static frame.
    declare_lidar_imu_frame_cmd = DeclareLaunchArgument(
        'lidar_imu_frame', default_value='camera_imu_optical_frame',
        description='Static frame the estimated body corresponds to. '
                    'camera_imu_optical_frame for l2_rsimu.yaml, '
                    'l2lidar_frame_imu for l2.yaml.'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 with both the raw FAST-LIO view and the loop-closure (PGO) view pre-configured '
                     '(/aft_pgo_map, /aft_pgo_path, /loop_closure_constraints).'
    )
    declare_rviz_cfg_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_cfg,
        description='RViz config file path'
    )
    # false, NOT true: this is the LIVE entry point, and 'true' on the robot
    # pins sim time at 0, so tf never resolves and nothing fuses, silently.
    # pepper_sensor_tf's publisher/scope are NOT derived from this -- on a bag
    # pass publisher:=none if it carries its own /tf_static, publisher:=urdf
    # scope:=all if it does not.
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
    # Height band for the 2D projection. 'map' is FLOOR-referenced (z=0 on the
    # ground plane), so these are plain heights above the floor.
    declare_occ_min_z_cmd = DeclareLaunchArgument(
        'occ_min_z', default_value='0.20',
        description='occupancy_min_z for octomap: height above the FLOOR (map '
                    'z=0 is the ground plane). Must stay above the floor or the '
                    'traversed floor is marked occupied (a black trail wherever the '
                    'robot drives), while still catching low obstacles.'
    )
    declare_occ_max_z_cmd = DeclareLaunchArgument(
        'occ_max_z', default_value='1.45',
        description='occupancy_max_z for octomap: height above the FLOOR. 1.45 m sits '
                    'just above Pepper, so nothing it cannot collide with is mapped.'
    )
    declare_self_hit_range_cmd = DeclareLaunchArgument(
        'self_hit_range', default_value='0.8',
        description='Drop scan points closer than this (m) before octomap, to remove '
                    'the low lidar seeing Pepper self-hits (a black trail along the path).'
    )
    declare_max_range_cmd = DeclareLaunchArgument(
        'max_range', default_value='20.0',
        description='octomap sensor_model.max_range (m). Beams are only inserted/cleared '
                    'out to this. Kept at full range for accuracy; clean the thin clearing-'
                    'ray spokes off the FINISHED map with clean_occupancy_map.py instead.'
    )
    # OFF by default: the per-scan L2 cloud is sparse, so any neighbourly
    # setting also deletes real far-wall returns (3-in-0.25 m dropped ~60% of
    # points), and it does not touch clearing-ray spokes anyway -- those need
    # clean_occupancy_map.py. Enable only for isolated occupied dots, and keep
    # it loose.
    declare_ror_neighbors_cmd = DeclareLaunchArgument(
        'ror_min_neighbors', default_value='0',
        description='Radius-outlier removal: drop scan points with fewer than this many '
                    'neighbours within ror_radius. 0 = OFF (default). Only for isolated '
                    'occupied dots; does NOT remove clearing-ray spokes.'
    )
    declare_ror_radius_cmd = DeclareLaunchArgument(
        'ror_radius', default_value='0.5',
        description='Radius (m) for radius-outlier removal neighbour count (keep large '
                    'on the sparse per-scan cloud so real far returns are not deleted).'
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

    # Static rig TF. The bridge needs base_footprint -> l2lidar_frame_imu from
    # here to close odom -> base_footprint. Only use_sim_time is forwarded, so
    # scope keeps its own 'mount' default -- right on the robot, where the
    # driver publishes the camera edges itself. See use_sim_time above for what
    # to pass on a bag.
    sensor_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensor_tf_share, 'launch', 'pepper_sensor_tf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # FAST-LIO owns lio_init -> base_footprint (via lio_odom_bridge).
    # bridge_level_frame:='false' disables the bridge's own leveled frame here
    # because PGO owns pgo_init -> lio_init below; otherwise lio_init
    # would get two parents. The leveling still happens, one level up:
    # pgo_map_odom_bridge publishes map -> pgo_init, so the leveled frame in
    # this stack is 'map' rather than 'odom'.
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('lio_config_file'),
            'rviz': rviz,
            'rviz_cfg': rviz_cfg,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # PGO owns the loop-closure correction map -> odom, completing the REP-105
    # tree:  map -> odom -> base_footprint -> l2lidar_frame -> l2lidar_frame_imu.
    # It also publishes the one-time upright frame map (RViz fixed frame).
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
            'lidar_imu_frame': LaunchConfiguration('lidar_imu_frame'),
            # MUST match the LIO config's publish.body_frame. level_source
            # 'calibration' builds the map -> pgo_init levelling rotation from
            # base_frame -> lidar_imu_frame, so leaving it at the node's
            # l2lidar_frame_imu default while FAST-LIO estimates the RealSense
            # IMU levels the whole map by the WRONG mount -- the two differ by
            # the camera's optical rotation, so the cloud comes out pointing
            # about 90 deg off with no error anywhere.
        }],
    )

    # Strips the robot self-hits the low lidar sees (they print as a black trail
    # along the path) before octomap. Only octomap consumes the filtered cloud;
    # SLAM is untouched.
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

    # Ray-traced 2D grid on /projected_map. Consumes the PER-SCAN cloud so it
    # can clear free space by casting rays from each sensor origin, which a
    # top-down projection of the accumulated /aft_pgo_map cannot do. Built in
    # the gravity-aligned map frame so the projection is level.
    # NOTE: octomap does not retro-correct inserted voxels, so a LARGE loop
    # closure leaves a seam. Fine for room/corridor drift.
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
            # Drop isolated single occupied voxels (stray beam noise / reflections /
            # points seen through wall gaps) so they don't pepper the map outside walls.
            'filter_speckles': True,
            'occupancy_min_z': occ_min_z,
            'occupancy_max_z': occ_max_z,
            # Segment the floor as ground (free) instead of obstacle. The map has
            # a residual ~1.5 deg tilt (FAST-LIO pitch drift), so a fixed z-band
            # alone can't separate floor from low obstacles across the whole map;
            # plane segmentation classifies the ground locally regardless of tilt.
            # Runs in base_frame_id (base_footprint), which sits ON the floor.
            'filter_ground_plane': True,
            'ground_filter.distance': 0.05,       # inlier band around fitted plane
            'ground_filter.angle': 0.15,          # rad; tolerant of the ~1.5 deg tilt
            'ground_filter.plane_distance': 0.12,  # plane must be within this of base z=0
            'latch': True,
        }],
    )

    # Do NOT add an LD_LIBRARY_PATH override here. One used to be needed for a
    # stale apt GTSAM; that package is gone and pgo_node links ros-humble-gtsam
    # 4.2.0 directly. An override now picks up the differently-built 4.2.0 at
    # /usr/local/lib instead (undefined symbol on NonlinearFactor::rekey, a TBB
    # config mismatch). If it crashes again, check `ldd pgo_node | grep gtsam`.
    pgo_node = Node(
        package='fastlio_lc_pgo',
        executable='pgo_node',
        name='laserPGO',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'save_directory': save_directory,
            'map_pcd_path': LaunchConfiguration('map_pcd_path'),
            # MUST match pgo_map_odom_bridge's level_frame ('map'); pgo_node's
            # own default is 'map_level'. When they disagree it warns and saves
            # the .pcd in the raw pgo_init frame -- ~90 deg off gravity -- while
            # still returning success=True. Recover such a map with
            # utils/build_pgo_map.py --level-tf.
            'level_frame': 'map',
            'keyframe_filter_size': LaunchConfiguration('keyframe_filter_size'),
            'cloud_topic': '/cloud_registered_body',
            'odom_topic': '/odom_lio',

            # PGO's optimized map/path/odom live in the loop-closure-corrected
            # "map" frame; pgo_map_odom_bridge turns the offset between this and
            # FAST-LIO's odom into the REP-105 map -> odom transform.
            'map_frame': 'pgo_init',

            # planar-motion prior -- see declare_planar_prior_cmd above
            'planar_prior_enable': ParameterValue(
                LaunchConfiguration('planar_prior'), value_type=bool),
            'planar_gravity_auto': ParameterValue(
                LaunchConfiguration('planar_gravity_auto'), value_type=bool),
            'planar_gravity': ParameterValue(
                LaunchConfiguration('planar_gravity'),
                value_type=List[float]),
            'planar_sigma_h': ParameterValue(
                LaunchConfiguration('planar_sigma_h'), value_type=float),

            # keyframe selection
            'keyframe_meter_gap': 1.0,
            'keyframe_deg_gap': 10.0,

            # Scan Context (indoor, Unitree L2: 30 m det_range -> same
            # indoor radius as the corridor benchmark run)
            'sc_dist_thres': 0.4,
            'sc_max_radius': 20.0,
            # Sensor height above the floor. Upstream hardcoded 2.0 (a car/
            # handheld value) inside SCManager; Pepper carries the L2 at 0.2582 m
            # per pepper_slam/config/sensor_tf.yaml. Wrong values here skew the
            # height binning the descriptor is built from.
            'sc_lidar_height': 0.2582,
            # OFF: Scan Context's failure mode is self-similar geometry, and a
            # repetitive indoor corridor is exactly that. It covers what the
            # radius search structurally cannot -- a revisit once drift exceeds
            # historyKeyframeSearchRadius -- and its candidates still face the
            # same ICP fitness test. Turn on if closures are missed after long
            # open-loop stretches.
            'use_scan_context': False,

            # loop closure
            # 1.5 m is tight, but detection now runs on the OPTIMISED poses, so
            # it no longer has to absorb the whole accumulated drift.
            'historyKeyframeSearchRadius': 1.5,
            # 30 s of Pepper's indoor travel can be well under 10 m, so short
            # values let near-in-time revisits (pausing, turning on the spot)
            # register as "loops". Raise if the graph fills with trivial edges.
            'historyKeyframeSearchTimeDiff': 30.0,
            'historyKeyframeSearchNum': 20,
            'speedFactor': 1.0,
            'loopClosureFrequency': 4.0,
            'graphUpdateFrequency': 2.0,
            'graphUpdateTimes': 5,
            # 0.01, not the original 0.1: each odometry edge is 1e-6 rad^2 /
            # 1e-4 m^2, so a 100-keyframe loop accumulates ~1e-4 / 1e-2. At 0.1
            # the loop factor was looser than the chain it had to correct and
            # iSAM2 barely moved the graph. Note robustLoopNoise wraps this in a
            # Cauchy kernel, so the realised correction is softer than the raw
            # variance ratio suggests -- and tightening further can therefore
            # HURT, by making each residual larger in sigmas.
            #
            # Splitting rotation from translation (Rot/Trans below, -1.0 = fall
            # back to the uniform value) was measured at 1e-4 / 1e-2 and did not
            # beat the uniform 0.01. That test is CONFOUNDED -- FAST-LIO replay
            # is nondeterministic, so the two arms saw different odometry and
            # run-to-run variance exceeded the effect. Inconclusive, not
            # evidence against. To settle it, record /odom_lio and
            # /cloud_registered_body once and replay that fixed input per arm.
            'loopNoiseScoreRot': -1.0,
            'loopNoiseScoreTrans': -1.0,
            'loopNoiseScore': 0.01,
            # Low on purpose: pubMap() rebuilds the ENTIRE map every call while
            # holding mKF. At 10 Hz and a few hundred keyframes it spins
            # continuously holding the lock, starving keyframe insertion and the
            # ICP thread while the buffers grow unbounded.
            'vizmapFrequency': 0.1,
            'loopFitnessScoreThreshold': 0.3,
            'mapviz_filter_size': mapviz_filter_size,
            'map_save_filter_size': map_save_filter_size,
        }],
    )
    # odom -> base_footprint. FAST_LIO's mapping.launch.py no longer starts this
    # (see FAST_LIO d8b274c): it is Pepper glue, and lived in a launch file
    # shared with every other FAST-LIO sensor config.
    lio_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pepper_slam'),
                         'launch', 'lio_odom_bridge.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': LaunchConfiguration('lio_config_file'),
            'lidar_imu_frame': LaunchConfiguration('lidar_imu_frame'),
            'bridge_level_frame': 'false',
        }.items())


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
    ld.add_action(lio_bridge_launch)
    ld.add_action(fast_lio_launch)
    ld.add_action(pgo_node)
    ld.add_action(pgo_map_odom_bridge)
    ld.add_action(range_filter_node)
    ld.add_action(octomap_node)
    return ld
