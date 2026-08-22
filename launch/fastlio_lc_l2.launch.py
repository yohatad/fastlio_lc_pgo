import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


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
        'save_directory', default_value='/home/yoha/Lidar/run_l2_lc/pgo_output/',
        description='Directory where PGO writes optimized poses, odom poses, times and keyframe scans (its Scans/ subfolder is wiped on startup)'
    )

    # The finished 3D map goes NEXT TO THE 2D GRID it shares a frame with, in
    # pepper_navigation/map -- not into save_directory, which is scratch (this
    # node wipes <save_directory>/Scans at startup and fills it with one .pcd
    # per keyframe plus pose logs). Written to the SOURCE tree so it survives a
    # rebuild; pepper_navigation's CMakeLists installs map/*.pcd to its share.
    declare_map_pcd_path_cmd = DeclareLaunchArgument(
        'map_pcd_path',
        default_value='/home/yoha/ros2_ws/src/pepper4dec/pepper_navigation/map/pepper_map_lc.pcd',
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
    # 2026-08-12: was hardcoded to l2.yaml, i.e. the L2's own IMU. That gyro
    # cancels rotation about the gravity axis below ~16 deg/s and cost 139 deg
    # of heading over a 744 s run (utils/L2_IMU/REPORT.md), so loop closure was
    # being asked to repair odometry with a known, large, systematic yaw error.
    # l2_rsimu.yaml drives the same estimator from the RealSense IMU instead;
    # measured 3.8% -> 2.4% mean yaw error, 11.2% -> 4.6% worst.
    declare_lio_config_file_cmd = DeclareLaunchArgument(
        'lio_config_file', default_value='l2_rsimu.yaml',
        description='FAST-LIO config. l2_rsimu.yaml uses the RealSense IMU '
                    '(recommended); l2.yaml uses the L2 s own.'
    )
    # Must match the IMU the config selects, or lio_map_odom_bridge closes
    # odom -> base_footprint through the wrong static frame.
    declare_lidar_imu_frame_cmd = DeclareLaunchArgument(
        'lidar_imu_frame', default_value='camera_imu_optical_frame',
        description='Static frame the estimated body corresponds to. '
                    'camera_imu_optical_frame for l2_rsimu.yaml, '
                    'l2lidar_frame_imu for l2.yaml.'
    )
    declare_sensor_tf_scope_cmd = DeclareLaunchArgument(
        'sensor_tf_scope', default_value='all', choices=['mount', 'all'],
        description="'all' publishes the camera edges too, needed for bag "
                    "replay where no RealSense driver is running. Use 'mount' "
                    "on the real robot so the driver's own values win."
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 with both the raw FAST-LIO view and the loop-closure (PGO) view pre-configured '
                     '(/aft_pgo_map, /aft_pgo_path, /loop_closure_constraints).'
    )
    declare_rviz_cfg_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_cfg,
        description='RViz config file path'
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
    # Height band for the 2D projection, in the map (gravity-aligned)
    # frame. map is now FLOOR-referenced (level_frame_on_floor in
    # pgo_map_odom_bridge puts its z=0 on the ground plane instead of at the
    # LIO start pose ~lidar-mount height up), so these are plain heights above
    # the floor. They used to be -0.25/1.0 against a floor measured at
    # ~-0.45 m; the same physical band is now 0.20/1.45.
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
    # Radius-outlier removal is OFF by default: the per-scan L2 cloud is sparse,
    # so any neighbourly setting also deletes real far-wall returns (tested
    # 3-in-0.25 m dropped ~60% of points), and it does NOT remove the clearing-
    # ray spokes anyway (those come from real see-through observations, not spike
    # endpoints -- use clean_occupancy_map.py for spokes). Enable ONLY if you see
    # isolated occupied dots, and keep it loose (e.g. ror_radius:=0.5).
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

    # Static sensor rig: base_footprint -> l2lidar_frame -> l2lidar_frame_imu (+ cams).
    # This is the piece missing from the bag's own /tf; without it the FAST-LIO
    # bridge cannot close odom -> base_footprint.
    # scope:=all is required for BAG REPLAY. pepper_sensor_tf's default 'mount'
    # scope omits the owner:driver camera edges on the assumption the RealSense
    # driver is publishing them live; with no driver the tree comes up as
    # disconnected islands and camera_imu_optical_frame -- which l2_rsimu.yaml
    # names as the body frame -- does not resolve at all. Use 'mount' on the
    # real robot so the driver's device-read values win.
    sensor_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensor_tf_share, 'launch', 'pepper_sensor_tf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time,
                          'scope': LaunchConfiguration('sensor_tf_scope')}.items()
    )

    # FAST-LIO owns odom_lidar -> base_footprint (via lio_map_odom_bridge).
    # bridge_level_frame:='false' disables the bridge's own leveled frame here
    # because PGO owns map_lidar -> odom_lidar below; otherwise odom_lidar
    # would get two parents. The leveling still happens, one level up:
    # pgo_map_odom_bridge publishes map -> map_lidar, so the leveled frame in
    # this stack is 'map' rather than 'odom'.
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('lio_config_file'),
            'lidar_imu_frame': LaunchConfiguration('lidar_imu_frame'),
            'rviz': rviz,
            'rviz_cfg': rviz_cfg,
            'use_sim_time': use_sim_time,
            'bridge_level_frame': 'false',
        }.items()
    )

    # PGO owns the loop-closure correction map -> odom, completing the REP-105
    # tree:  map -> odom -> base_footprint -> l2lidar_frame -> l2lidar_frame_imu.
    # It also publishes the one-time upright frame map (RViz fixed frame).
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
            'odom_topic': '/odom_lio',
            'pgo_odom_topic': '/aft_pgo_odom',
            'publish_level_frame': True,
            # MUST match the LIO config's publish.body_frame. level_source
            # 'calibration' builds the map -> map_lidar levelling rotation from
            # base_frame -> lidar_imu_frame, so leaving it at the node's
            # l2lidar_frame_imu default while FAST-LIO estimates the RealSense
            # IMU levels the whole map by the WRONG mount -- the two differ by
            # the camera's optical rotation, so the cloud comes out pointing
            # about 90 deg off with no error anywhere.
            'lidar_imu_frame': LaunchConfiguration('lidar_imu_frame'),
        }],
    )

    # Ray-traced 2D occupancy grid via octomap_server. It consumes the PER-SCAN
    # cloud /cloud_registered_body (body frame l2lidar_frame_imu) and, using our
    # corrected TF (map -> odom -> base_footprint -> l2lidar_frame_imu), clears free
    # space by casting rays from each scan's sensor origin -- which a top-down
    # projection of the accumulated /aft_pgo_map cannot do. The octree is built
    # in the gravity-aligned map frame so the down-projection is level.
    # Publishes nav_msgs/OccupancyGrid on /projected_map.
    # NOTE: octomap inserts scans online and does not retro-correct already-
    # inserted voxels, so a LARGE loop closure leaves a seam; rebuild from
    # corrected keyframes if that matters. For room/corridor drift it is fine.
    # Strip robot self-hits (near-field cluster within ~0.6 m of the low lidar)
    # from the scan before octomap; otherwise they print as a black trail along
    # the path. Only octomap consumes the filtered cloud; SLAM is untouched.
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

    # Historical note: this used to need an LD_LIBRARY_PATH override to avoid
    # loading an ABI-incompatible GTSAM at runtime (a stale apt libgtsam4
    # 4.1.1 the binary was originally compiled against, vs. ros-humble-gtsam
    # 4.2.0 installed alongside it). That apt package no longer exists on
    # this system at all -- confirmed via `dpkg -l | grep gtsam` finding only
    # ros-humble-gtsam, and a rebuild of this package now links pgo_node
    # against ros-humble-gtsam 4.2.0 directly (confirmed via `ldd`), with no
    # override needed. Re-adding an LD_LIBRARY_PATH override here would
    # actively BREAK it again: this machine also has a second, differently-
    # built GTSAM 4.2.0 at /usr/local/lib (undefined symbol on
    # NonlinearFactor::rekey when loaded instead -- a TBB build-config
    # mismatch, not a version mismatch). If this starts crashing again after
    # a future system change, check `ldd pgo_node | grep gtsam` before
    # reaching for an env override -- confirm which GTSAM it actually needs
    # first.
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
            # silently saves the .pcd in the RAW map_lidar frame instead --
            # ~90 deg off gravity (measured -92.2 deg roll / -88.1 deg yaw on
            # 2026-08-22), which does not match the 2D grid octomap builds in
            # 'map'. The service still returns success=True, so the only clue
            # is the "frame map_lidar" in its message. Recover a map already
            # saved that way with utils/build_pgo_map.py --level-tf.
            'level_frame': 'map',
            'keyframe_filter_size': LaunchConfiguration('keyframe_filter_size'),
            'cloud_topic': '/cloud_registered_body',
            'odom_topic': '/odom_lio',

            # PGO's optimized map/path/odom live in the loop-closure-corrected
            # "map" frame; pgo_map_odom_bridge turns the offset between this and
            # FAST-LIO's odom into the REP-105 map -> odom transform.
            'map_frame': 'map_lidar',

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
            # performSCLoopClosure() was dead code until 2026-08-12; descriptors
            # were built for every keyframe and never read. Now callable, but
            # OFF by default: Scan Context's known failure mode is self-similar
            # geometry, and a repetitive indoor corridor is exactly that. Its
            # value is covering what the radius search structurally cannot --
            # a revisit once drift already exceeds historyKeyframeSearchRadius.
            # Every candidate it proposes still faces the same ICP fitness test.
            # Turn on if closures are being missed after long open-loop stretches.
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
            # 2026-08-12: 0.1 -> 0.01. initNoises() gives each odometry edge
            # 1e-6 rad^2 / 1e-4 m^2, so over a 100-keyframe loop the chain
            # accumulates ~1e-4 rad^2 / 1e-2 m^2. A loop factor at 0.1 was one
            # to three orders LOOSER than the chain it was meant to correct, so
            # iSAM2 kept the odometry and barely moved the graph -- worst for
            # rotation, which is the dominant indoor error mode. 0.01 puts the
            # loop factor at the chain's translational scale.
            # NB robustLoopNoise wraps this in a Cauchy m-estimator, which
            # downweights large residuals further, so the effective correction
            # is softer than the raw variance ratio suggests. Validate on a run
            # with a known revisit before trusting the number.
            # Split rotation from translation, mirroring odomNoise's own 100x
            # split (1e-6 rad^2 / 1e-4 m^2 per edge). A single scalar across all
            # six DOF cannot sit correctly against both: MEASURED on this bag
            # with the L2 IMU at a uniform 0.01, translation got ~50% of the
            # loop error but rotation only ~1%, and the endpoint improved just
            # 0.735 -> 0.579 m. Each value now sits at its own chain's
            # accumulated scale over a ~100-keyframe loop.
            # loopNoiseScore below is the legacy uniform fallback, unused while
            # both of these are positive.
            # MEASURED and NOT adopted: 1e-4 / 1e-2 was tried on this bag with
            # l2.yaml and did NOT beat the uniform 0.01 -- correction fell to
            # 0.330 m / 2.49 deg from 0.475 m / 2.76 deg, and the endpoint gained
            # only 1.7% against 21.1%. That comparison is CONFOUNDED, though: the
            # two runs' RAW odometry differed (end error 0.735 vs 0.985 m, path
            # 165.37 vs 165.73 m) because FAST-LIO replay is nondeterministic, so
            # PGO saw different input and the run-to-run variance exceeds the
            # effect. Treat the numbers as inconclusive, not as evidence against.
            #
            # If the effect is real, the likely cause is the Cauchy kernel:
            # tightening the variance makes each residual larger IN SIGMAS, and
            # Cauchy downweights large normalised residuals harder, so
            # over-trusting a loop factor can reduce its realised influence.
            #
            # To settle it, remove the nondeterminism: record FAST-LIO's
            # /odom_lio and /cloud_registered_body ONCE, then replay that fixed
            # recording into PGO for each candidate setting, so every arm sees
            # byte-identical input.
            #
            # -1.0 means "fall back to loopNoiseScore", i.e. the uniform value.
            'loopNoiseScoreRot': -1.0,
            'loopNoiseScoreTrans': -1.0,
            'loopNoiseScore': 0.01,
            # 2026-08-12: 10.0 -> 0.1 (the node's own default). pubMap() rebuilds
            # the ENTIRE map every call -- local2global over every keyframe, then
            # a voxel filter over the whole accumulated cloud -- while holding
            # mKF. At 10 Hz and a few hundred keyframes it cannot keep up, so it
            # spins continuously holding the lock, starving keyframe insertion,
            # updatePoses and the ICP thread while odometryBuf/fullResBuf grow
            # unbounded.
            'vizmapFrequency': 0.1,
            'loopFitnessScoreThreshold': 0.3,
            'mapviz_filter_size': mapviz_filter_size,
            'map_save_filter_size': map_save_filter_size,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_save_directory_cmd)
    ld.add_action(declare_map_pcd_path_cmd)
    ld.add_action(declare_keyframe_filter_size_cmd)
    ld.add_action(declare_lio_config_file_cmd)
    ld.add_action(declare_lidar_imu_frame_cmd)
    ld.add_action(declare_sensor_tf_scope_cmd)
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
    ld.add_action(fast_lio_launch)
    ld.add_action(pgo_node)
    ld.add_action(pgo_map_odom_bridge)
    ld.add_action(range_filter_node)
    ld.add_action(octomap_node)
    return ld
