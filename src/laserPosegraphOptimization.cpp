/**
 * @file laserPosegraphOptimization.cpp
 * @brief ROS2 port of yanliang-wang/FAST_LIO_LC's PGO/src/laserPosegraphOptimization.cpp
 *
 * 1. Detect the keyframes
 * 2. Maintain the Gtsam-based pose graph
 * 3. Detect the radius-search-based loop closure, and add them to the pose graph
 *
 * Ported from roscpp/tf to rclcpp/tf2. The GTSAM/PCL/ScanContext logic itself is
 * unchanged from the original - only the ROS I/O layer was translated.
 */
#include <fstream>
#include <sstream>
#include <iomanip>
#include <map>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <atomic>
#include <iostream>
#include <string>
#include <optional>
#include <filesystem>   // save_directory handling, replacing system("rm -r")
#include <cstdlib>      // getenv, for the save_directory default

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

#include "scancontext/Scancontext.h"

using namespace gtsam;

using std::cout;
using std::endl;

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0; // large value means must add the first given frame.
double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.

bool isNowKeyFrame = false;

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0}; // init
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0}; // init pose is zero

std::queue<nav_msgs::msg::Odometry::SharedPtr> odometryBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> fullResBuf;
std::queue<sensor_msgs::msg::NavSatFix::SharedPtr> gpsBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds;
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<double> keyframeTimes;
int recentIdxUpdated = 0;
// for loop closure detection
std::map<int, int> loopIndexContainer; // existing loop pairs
pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<pcl::PointXYZ>());
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;

gtsam::NonlinearFactorGraph gtSAMgraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
SCManager scManager;
double scDistThres, scMaximumRadius, scLidarHeight;
bool useScanContext = false;

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
// Separate leaf size for the map_batch.pcd written by /pgo_batch_optimize.
// The saved map is a LOCALIZATION PRIOR and wants to be dense; the RViz map is
// republished every vizmapFrequency and wants to stay light. Sharing one filter
// forced a choice between a coarse prior and a laggy RViz.
pcl::VoxelGrid<PointType> downSizeFilterMapSave;
bool laserCloudMapPGORedraw = true;

bool useGPS = true;
sensor_msgs::msg::NavSatFix::SharedPtr currGPS;
bool hasGPSforThisKF = false;
bool gpsOffsetInitialized = false;
double gpsAltitudeInitOffset = 0.0;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal, pubLoopSubmapLocal;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomRepubVerifier;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srvBatchOptimize;

std::string save_directory;
std::string pgKITTIformat, pgScansDirectory;
std::string odomKITTIformat;
// World frame all PGO outputs (map, path, odom, loop markers, TF parent) are
// stamped in. Mirrors FAST-LIO's publish.map_frame parameter: defaults to the
// legacy "camera_init" but is overridden to "odom" so the optimized map lands
// in the same REP-105 tree the lio_map_odom_bridge already publishes.
std::string map_frame = "camera_init";
std::fstream pgTimeSaveStream;

// for front_end
rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pubKeyFramesId;

// for loop closure
double historyKeyframeSearchRadius;
double historyKeyframeSearchTimeDiff;
int historyKeyframeSearchNum;
double loopClosureFrequency;
int graphUpdateTimes;
double graphUpdateFrequency;
double loopNoiseScore;           // legacy uniform value; used only as the fallback for the split pair below
double loopNoiseScoreRot, loopNoiseScoreTrans;
double vizmapFrequency;
double vizPathFrequency;
double speedFactor;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocalRegisted;
double loopFitnessScoreThreshold;

rclcpp::Node::SharedPtr g_node;
std::shared_ptr<tf2_ros::TransformBroadcaster> g_tf_broadcaster;
// Used only at save time, to express map_batch.pcd in the gravity-aligned,
// floor-referenced level frame instead of the raw LIO-anchored map frame.
std::shared_ptr<tf2_ros::Buffer> g_tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> g_tf_listener;
std::string level_frame;
bool save_in_level_frame = true;
std::atomic<uint32_t> g_seq_counter{0};

// -- small ROS1 -> ROS2 time helpers --------------------------------------
inline double stampToSec(const builtin_interfaces::msg::Time & t)
{
    return rclcpp::Time(t).seconds();
}

inline builtin_interfaces::msg::Time secToStamp(double sec)
{
    rclcpp::Time t(static_cast<int64_t>(sec * 1e9));
    return t;
}

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

void saveOdometryVerticesKITTIformat(std::string _filename)
{
    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& _pose6d: keyframePoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);

    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void laserOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr _laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(_laserOdometry);
	mBuf.unlock();
} // laserOdometryHandler

void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::SharedPtr _laserCloudFullRes)
{
	mBuf.lock();
	fullResBuf.push(_laserCloudFullRes);
	mBuf.unlock();
} // laserCloudFullResHandler

void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr _gps)
{
    if(useGPS) {
        mBuf.lock();
        gpsBuf.push(_gps);
        mBuf.unlock();
    }
} // gpsHandler

void initNoises( void )
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    // LOCAL FIX: split rotation from translation, mirroring odomNoise above.
    //
    // Upstream applied one scalar to all six DOF while odomNoise splits them a
    // hundredfold (1e-6 rad^2 vs 1e-4 m^2), so no single value could sit
    // correctly against both. iSAM2 weights a loop factor against the chain it
    // has to overcome, roughly sigma2_chain / (sigma2_chain + sigma2_loop) over
    // an N-keyframe loop. MEASURED on bag/slam_august_8_bag with the L2 IMU
    // (which drifts, so there is real error to correct) at a uniform 0.01:
    //   translation  chain ~1e-2 vs loop 1e-2  -> ~50% corrected, 0.475 m mean
    //   rotation     chain ~1e-4 vs loop 1e-2  -> ~1%  corrected, 2.76 deg mean
    // and the endpoint moved 0.735 -> 0.579 m, i.e. only 21% of the loop error
    // came out -- yaw was still barely being touched, which is the dominant
    // indoor error mode.
    //
    // Defaults below put each component at its own chain's scale. Note
    // robustLoopNoise wraps these in a Cauchy m-estimator, which downweights
    // large residuals further, so the realised correction stays softer than the
    // raw variance ratio implies -- deliberately, since a false loop that
    // survived the ICP fitness gate must not be able to fold the map.
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScoreRot, loopNoiseScoreRot, loopNoiseScoreRot,
                          loopNoiseScoreTrans, loopNoiseScoreTrans, loopNoiseScoreTrans;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1),
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0;
    gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore;
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1),
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3) );

} // initNoises

Pose6D getOdom(nav_msgs::msg::Odometry::SharedPtr _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    const geometry_msgs::msg::Quaternion & quat = _odom->pose.pose.orientation;
    tf2::Quaternion tfq(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw, g_seq_counter.fetch_add(1)};
} // getOdom

Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles (SE3_delta, dx, dy, dz, droll, dpitch, dyaw);

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw)), 0};
} // SE3Diff

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

    int numberOfCores = 16;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void pubPath( void )
{
    // LOCAL FIX: bail before publishing anything when no keyframe has been
    // optimised yet. The loop below runs node_idx < recentIdxUpdated, so at
    // recentIdxUpdated == 0 it never executes and the DEFAULT-CONSTRUCTED
    // odomAftPGO went out: position (0,0,0), quaternion (0,0,0,0) -- not a unit
    // quaternion -- and stamp 0, followed by a TF carrying the same. Downstream
    // pgo_map_odom_bridge.py does not validate it: pose_to_matrix falls back to
    // identity rotation on the degenerate quaternion and latches a bogus
    // map_lidar -> odom_lidar correction until the next PGO update.
    if (recentIdxUpdated <= 0)
        return;

    // pub odom and path
    nav_msgs::msg::Odometry odomAftPGO;
    nav_msgs::msg::Path pathAftPGO;
    pathAftPGO.header.frame_id = map_frame;
    mKF.lock();
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++)
    {
        const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // updated poses

        nav_msgs::msg::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = map_frame;
        odomAftPGOthis.child_frame_id = "aft_pgo";
        odomAftPGOthis.header.stamp = secToStamp(keyframeTimes.at(node_idx));
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;
        tf2::Quaternion q;
        q.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGOthis.pose.pose.orientation = tf2::toMsg(q);
        odomAftPGO = odomAftPGOthis;

        geometry_msgs::msg::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = map_frame;
        pathAftPGO.poses.push_back(poseStampAftPGO);
    }
    mKF.unlock();
    pubOdomAftPGO->publish(odomAftPGO); // last pose
    pubPathAftPGO->publish(pathAftPGO); // poses

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odomAftPGO.header.stamp;
    transform.header.frame_id = map_frame;
    transform.child_frame_id = "aft_pgo";
    transform.transform.translation.x = odomAftPGO.pose.pose.position.x;
    transform.transform.translation.y = odomAftPGO.pose.pose.position.y;
    transform.transform.translation.z = odomAftPGO.pose.pose.position.z;
    transform.transform.rotation = odomAftPGO.pose.pose.orientation;
    g_tf_broadcaster->sendTransform(transform);
} // pubPath

void updatePoses(void)
{
    mKF.lock();
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
    {
        Pose6D& p =keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mKF.unlock();

    mtxRecentPose.lock();
    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
    recentOptimizedX = lastOptimizedPose.translation().x();
    recentOptimizedY = lastOptimizedPose.translation().y();

    recentIdxUpdated = int(keyframePosesUpdated.size()) - 1;

    mtxRecentPose.unlock();
} // updatePoses

void runISAM2opt(void)
{
    // called when a variable added
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    for(int i = graphUpdateTimes; i > 0; --i){
        isam->update();
    }

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
    pubPath();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
                                    transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(),
                                    transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw() );

    int numberOfCores = 8;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = root_idx + i;
        // LOCAL FIX: the bound was read outside mKF while the lock was taken
        // only for the access itself, so the size could change between the two.
        mKF.lock();
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) ) {
            mKF.unlock();
            continue;
        }
        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
        mKF.unlock();
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud

void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
{
    nearKeyframes->clear();
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        // LOCAL FIX: cloudSize was cached from an unlocked read before the loop.
        mKF.lock();
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) ) {
            mKF.unlock();
            continue;
        }
        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
        mKF.unlock();
    }

    if (nearKeyframes->empty())
        return;

    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

Eigen::Affine3f Pose6dToAffine3f(Pose6D pose)
{
    return pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
}

gtsam::Pose3 Pose6dTogtsamPose3(Pose6D pose)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(pose.roll), double(pose.pitch), double(pose.yaw)),
                                gtsam::Point3(double(pose.x),    double(pose.y),     double(pose.z)));
}

gtsam::Pose3 doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx )
{
    // parse pointclouds
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframes(cureKeyframeCloud, _curr_kf_idx, 0);
    loopFindNearKeyframes(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum);

    // loop verification
    sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = map_frame;
    pubLoopScanLocal->publish(cureKeyframeCloudMsg);

    sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = map_frame;
    pubLoopSubmapLocal->publish(targetKeyframeCloudMsg);

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    // LOCAL FIX: was a flat 150 m, against a 1.5 m detection radius. That let
    // ICP draw correspondences from anywhere in the +/-historyKeyframeSearchNum
    // submap, widening the basin for confident WRONG convergence. LIO-SAM uses
    // ~2x the search radius; scaling keeps the two coupled if the radius is
    // retuned. Floored so a very small radius cannot starve the first iteration.
    icp.setMaxCorrespondenceDistance(std::max(2.0 * historyKeyframeSearchRadius, 5.0));
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    sensor_msgs::msg::PointCloud2 cureKeyframeCloudRegMsg;
    pcl::toROSMsg(*unused_result, cureKeyframeCloudRegMsg);
    cureKeyframeCloudRegMsg.header.frame_id = map_frame;
    pubLoopScanLocalRegisted->publish(cureKeyframeCloudRegMsg);

    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return gtsam::Pose3::Identity();
    } else {
        std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    // LOCAL FIX: both of these indexed keyframePosesUpdated without mKF.
    mKF.lock();
    Pose6D currPose = keyframePosesUpdated[_curr_kf_idx];
    Pose6D loopPose = keyframePosesUpdated[_loop_kf_idx];
    mKF.unlock();

    Eigen::Affine3f tWrong = Pose6dToAffine3f(currPose);

    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo =  Pose6dTogtsamPose3(loopPose);

    return poseFrom.between(poseTo);
} // doICPVirtualRelative

void process_pg()
{
    while(rclcpp::ok())
    {
		while ( !odometryBuf.empty() && !fullResBuf.empty() )
        {
            //
            // pop and check keyframe is or not
            //
			mBuf.lock();
            while (!odometryBuf.empty() && stampToSec(odometryBuf.front()->header.stamp) < stampToSec(fullResBuf.front()->header.stamp))
                odometryBuf.pop();
            if (odometryBuf.empty())
            {
                mBuf.unlock();
                break;
            }

            // Time equal check
            timeLaserOdometry = stampToSec(odometryBuf.front()->header.stamp);
            timeLaser = stampToSec(fullResBuf.front()->header.stamp);

            laserCloudFullRes->clear();
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
            fullResBuf.pop();

            Pose6D pose_curr = getOdom(odometryBuf.front());
            odometryBuf.pop();

            // find nearest gps
            double eps = 0.1; // find a gps topic arrived within eps second
            while (!gpsBuf.empty()) {
                auto thisGPS = gpsBuf.front();
                auto thisGPSTime = stampToSec(thisGPS->header.stamp);
                if( abs(thisGPSTime - timeLaserOdometry) < eps ) {
                    currGPS = thisGPS;
                    hasGPSforThisKF = true;
                    break;
                } else {
                    hasGPSforThisKF = false;
                }
                gpsBuf.pop();
            }
            mBuf.unlock();

            //
            // Early reject by counting local delta movement (for equi-sperated kf drop)
            //
            odom_pose_prev = odom_pose_curr;
            odom_pose_curr = pose_curr;
            Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform

            double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z); // note: absolute value.
            translationAccumulated += delta_translation;
            rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.

            // keyframe selection
            if( translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap ) {
                isNowKeyFrame = true;
                translationAccumulated = 0.0; // reset
                rotaionAccumulated = 0.0; // reset
            } else {
                isNowKeyFrame = false;
            }

            if( ! isNowKeyFrame )
                continue;

            if( !gpsOffsetInitialized ) {
                if(hasGPSforThisKF) { // if the very first frame
                    gpsAltitudeInitOffset = currGPS->altitude;
                    gpsOffsetInitialized = true;
                }
            }

            //
            // Save data and Add consecutive node
            //
            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            downSizeFilterScancontext.setInputCloud(thisKeyFrame);
            downSizeFilterScancontext.filter(*thisKeyFrameDS);

            mKF.lock();
            keyframeLaserClouds.push_back(thisKeyFrameDS);
            keyframePoses.push_back(pose_curr);
            {
                // publish keyframe id
                std_msgs::msg::Header keyFrameHeader;
                keyFrameHeader.stamp = g_node->now();
                pubKeyFramesId->publish(keyFrameHeader);
            }
            keyframePosesUpdated.push_back(pose_curr); // init
            keyframeTimes.push_back(timeLaserOdometry);

            scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);

            laserCloudMapPGORedraw = true;
            mKF.unlock();

            const int prev_node_idx = keyframePoses.size() - 2;
            const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts with 0
            if( ! gtSAMgraphMade /* prior node */) {
                const int init_node_idx = 0;
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));

                mtxPosegraph.lock();
                {
                    // prior factor
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                    initialEstimate.insert(init_node_idx, poseOrigin);
                }
                mtxPosegraph.unlock();

                gtSAMgraphMade = true;

                cout << "posegraph prior node " << init_node_idx << " added" << endl;
            } else /* consecutive node (and odom factor) after the prior added */ {
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                mtxPosegraph.lock();
                {
                    // odom factor
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odomNoise));

                    // gps factor
                    if(hasGPSforThisKF) {
                        double curr_altitude_offseted = currGPS->altitude - gpsAltitudeInitOffset;
                        mtxRecentPose.lock();
                        gtsam::Point3 gpsConstraint(recentOptimizedX, recentOptimizedY, curr_altitude_offseted);
                        mtxRecentPose.unlock();
                        gtSAMgraph.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise));
                        cout << "GPS factor added at node " << curr_node_idx << endl;
                    }
                    initialEstimate.insert(curr_node_idx, poseTo);
                }
                mtxPosegraph.unlock();

                if(curr_node_idx % 100 == 0)
                    cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }

            // save utility
            std::string curr_node_idx_str = padZeros(curr_node_idx);
            pcl::io::savePCDFileBinary(pgScansDirectory + curr_node_idx_str + ".pcd", *thisKeyFrame); // scan
            pgTimeSaveStream << timeLaser << std::endl; // path
        }

        // ps.
        // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_pg

// Place-recognition loop detection, complementary to the radius search: it
// matches on APPEARANCE, so it can still fire when drift already exceeds
// historyKeyframeSearchRadius -- the case the radius search structurally
// cannot cover. Gated on use_scan_context because Scan Context's known
// weakness is self-similar geometry, and a repetitive indoor corridor is
// exactly that; every candidate it proposes still has to pass the same ICP
// fitness test, so a false proposal costs time rather than correctness.
void performSCLoopClosure(void)
{
    // LOCAL FIX: size/back() read without mKF while process_pg appends.
    mKF.lock();
    const int numKeyframes = int(keyframePoses.size());
    mKF.unlock();
    if( numKeyframes < scManager.NUM_EXCLUDE_RECENT) // do not try too early
        return;

    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
    int SCclosestHistoryFrameID = detectResult.first;
    if( SCclosestHistoryFrameID != -1 ) {
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = numKeyframes - 1; // because cpp starts 0 and ends n-1
        mBuf.lock();
        // Skip if the radius search already has an accepted constraint here.
        if (loopIndexContainer.find(curr_node_idx) != loopIndexContainer.end()) {
            mBuf.unlock();
            return;
        }
        cout << "[SC] Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << endl;
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // adding actual 6D constraints in the other thread, icp_calculation.
        mBuf.unlock();
    }
} // performSCLoopClosure

pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc(const std::vector<Pose6D> vectorPose6d){
    pcl::PointCloud<pcl::PointXYZ>::Ptr res( new pcl::PointCloud<pcl::PointXYZ> ) ;
    for( auto p : vectorPose6d){
        res->points.emplace_back(p.x, p.y, p.z);
    }
    return res;
}

/**
 * Find near keyframes by distance in history keyframes; pick a temporally distant one as loop candidate
*/
bool detectLoopClosureDistance(int *loopKeyCur, int *loopKeyPre)
{
    auto it = loopIndexContainer.find(*loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    // LOCAL FIX: search the OPTIMISED poses, not the raw odometry chain.
    // keyframePoses is never touched by iSAM2, so detection saw the full
    // accumulated drift: with historyKeyframeSearchRadius 1.5 m a revisit was
    // only found once drift was already under 1.5 m -- i.e. precisely when loop
    // closure was not needed. It was also inconsistent with
    // doICPVirtualRelative(), which builds its submaps from keyframePosesUpdated
    // (see :588/:593), so detection and verification disagreed about where the
    // robot was. Upstream LIO-SAM / FAST_LIO_LC both search the corrected poses.
    mKF.lock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloudKeyPoses3D = vector2pc(keyframePosesUpdated);
    std::vector<double> copy_keyframeTimes = keyframeTimes;
    mKF.unlock();
    if (copy_cloudKeyPoses3D->empty()) return false;
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

    for(int i = 0; i < pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        if ( id >= int(copy_keyframeTimes.size()) || *loopKeyCur >= int(copy_keyframeTimes.size()) )
            continue;
        if ( abs( copy_keyframeTimes[id] - copy_keyframeTimes[*loopKeyCur] ) > historyKeyframeSearchTimeDiff )
        {
            *loopKeyPre = id;
            break;
        }
    }

    if (*loopKeyPre == -1 || *loopKeyCur == *loopKeyPre)
        return false;

    return true;
}

void performRSLoopClosure(void)
{
    // LOCAL FIX: size/back() were read without mKF while process_pg push_backs
    // into these vectors. A reallocation mid-read is a use-after-free.
    mKF.lock();
    const int numKeyframes = int(keyframePoses.size());
    mKF.unlock();
    if( numKeyframes == 0 )
        return;

    int loopKeyCur = numKeyframes - 1;
    int loopKeyPre = -1;
    if ( detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) ){
        cout << "Loop detected! - between " << loopKeyPre << " and " << loopKeyCur << "" << endl;
        mBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(loopKeyPre, loopKeyCur));
        mBuf.unlock();
        // LOCAL FIX: the loopIndexContainer insert used to happen HERE, before
        // ICP had verified anything. detectLoopClosureDistance()'s early-out
        // treats any key present in that map as already handled, so a candidate
        // that ICP then rejected on fitness blacklisted its keyframe FOREVER --
        // and visualizeLoopClosure() drew a marker edge for a constraint that
        // was never added to the graph. It is now inserted in process_icp()
        // only after the fitness test passes.
    } else
        return;
} // performRSLoopClosure

void visualizeLoopClosure()
{
    // LOCAL FIX: loopIndexContainer, keyframeTimes and keyframePosesUpdated
    // were all read here with no lock while process_pg / process_icp were
    // writing them. Snapshot once under the locks, then build markers from the
    // copies -- a reallocating push_back mid-iteration is a use-after-free.
    mBuf.lock();
    std::map<int, int> loopPairs = loopIndexContainer;
    mBuf.unlock();
    if (loopPairs.empty())
        return;

    mKF.lock();
    if (keyframeTimes.empty()) { mKF.unlock(); return; }
    const double lastKeyframeTime = keyframeTimes.back();
    std::vector<Pose6D> posesSnapshot = keyframePosesUpdated;
    mKF.unlock();

    visualization_msgs::msg::MarkerArray markerArray;
    visualization_msgs::msg::Marker markerNode;
    markerNode.header.frame_id = map_frame;
    markerNode.header.stamp = secToStamp(lastKeyframeTime);
    markerNode.action = visualization_msgs::msg::Marker::ADD;
    markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3;
    markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
    markerNode.color.a = 1;
    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = map_frame;
    markerEdge.header.stamp = secToStamp(lastKeyframeTime);
    markerEdge.action = visualization_msgs::msg::Marker::ADD;
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    for (auto it = loopPairs.begin(); it != loopPairs.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        // The snapshot can lag loopPairs by an insert; skip rather than index OOB.
        if (key_cur < 0 || key_pre < 0 ||
            key_cur >= int(posesSnapshot.size()) || key_pre >= int(posesSnapshot.size()))
            continue;
        geometry_msgs::msg::Point p;
        p.x = posesSnapshot[key_cur].x;
        p.y = posesSnapshot[key_cur].y;
        p.z = posesSnapshot[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = posesSnapshot[key_pre].x;
        p.y = posesSnapshot[key_pre].y;
        p.z = posesSnapshot[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge->publish(markerArray);
}

void process_lcd(void)
{
    rclcpp::Rate rate(loopClosureFrequency);
    while (rclcpp::ok())
    {
        rate.sleep();
        performRSLoopClosure();
        // LOCAL FIX: performSCLoopClosure() existed but was never called, so
        // makeAndSaveScancontextAndKeys() built a descriptor for every keyframe
        // that nothing ever read, and sc_dist_thres / sc_max_radius were inert.
        if (useScanContext) performSCLoopClosure();
        visualizeLoopClosure();
    }
} // process_lcd

void process_icp(void)
{
    while(rclcpp::ok())
    {
		while ( !scLoopICPBuf.empty() )
        {
            if( scLoopICPBuf.size() > 30 ) {
                RCLCPP_WARN(g_node->get_logger(), "Too many loop closure candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }

            mBuf.lock();
            std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
            scLoopICPBuf.pop();
            mBuf.unlock();

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            auto relative_pose = doICPVirtualRelative(prev_node_idx, curr_node_idx);
            if( !relative_pose.equals( gtsam::Pose3::Identity() )) {
                mtxPosegraph.lock();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(curr_node_idx, prev_node_idx, relative_pose, robustLoopNoise));
                mtxPosegraph.unlock();
                // Record the accepted pair only now -- see performRSLoopClosure().
                // A rejected candidate must stay eligible so a later revisit,
                // with better overlap, can be retried.
                mBuf.lock();
                loopIndexContainer[curr_node_idx] = prev_node_idx;
                mBuf.unlock();
            }
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_icp

// NOT STARTED by main(), and deliberately left that way. pubPath() is already
// called from process_isam() after every optimisation, so /aft_pgo_odom and
// /aft_pgo_path update at graphUpdateFrequency (2 Hz) -- running this thread as
// well would publish the same poses twice and double the TF broadcast on
// map_frame -> aft_pgo. vizPathFrequency is therefore unused. Kept for upstream
// diff parity; delete both if that stops mattering.
void process_viz_path(void)
{
    float hz = vizPathFrequency;
    rclcpp::Rate rate(hz);
    while (rclcpp::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubPath();
        }
    }
}

void process_isam(void)
{
    float hz = graphUpdateFrequency;
    rclcpp::Rate rate(hz);
    while (rclcpp::ok()) {
        rate.sleep();
        if( gtSAMgraphMade ) {
            mtxPosegraph.lock();
            runISAM2opt();
            mtxPosegraph.unlock();

            saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat); // pose
            saveOdometryVerticesKITTIformat(odomKITTIformat); // pose
        }
    }
}

void pubMap(void)
{
    int SKIP_FRAMES = 1; // sparse map visualization to save computations
    int counter = 0;

    laserCloudMapPGO->clear();

    mKF.lock();
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) {
        if(counter % SKIP_FRAMES == 0) {
            *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
        }
        counter++;
    }
    mKF.unlock();

    downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
    downSizeFilterMapPGO.filter(*laserCloudMapPGO);

    sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = map_frame;
    pubMapAftPGO->publish(laserCloudMapPGOMsg);
}

// One-shot batch re-optimization of the FULL accumulated pose graph, called
// via the /pgo_batch_optimize service once a session is done. process_isam()
// only ever runs iSAM2's incremental update (good enough given data seen so
// far); ISAM2 internally retains every factor it has been given, so
// getFactorsUnsafe() hands back the complete NonlinearFactorGraph built over
// the whole run, which a batch Levenberg-Marquardt solve can then optimize
// as a single global problem instead of walking through it causally.
// Writes optimized_poses_batch.txt (does not touch the live
// optimized_poses.txt) and rebuilds+saves a map from the batch poses.
void batchOptimizeHandler(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    mtxPosegraph.lock();
    gtsam::NonlinearFactorGraph fullGraph = isam->getFactorsUnsafe();
    gtsam::Values initial = isam->calculateEstimate();
    mtxPosegraph.unlock();

    if (fullGraph.empty() || initial.empty()) {
        response->success = false;
        response->message = "Pose graph is empty, nothing to optimize.";
        return;
    }

    double initialError = fullGraph.error(initial);

    gtsam::LevenbergMarquardtParams params;
    params.setVerbosityLM("SUMMARY");
    params.setMaxIterations(100);
    gtsam::LevenbergMarquardtOptimizer optimizer(fullGraph, initial, params);
    gtsam::Values batchEstimate = optimizer.optimize();

    double finalError = fullGraph.error(batchEstimate);

    saveOptimizedVerticesKITTIformat(batchEstimate, save_directory + "optimized_poses_batch.txt");

    pcl::PointCloud<PointType>::Ptr batchMap(new pcl::PointCloud<PointType>());
    mKF.lock();
    int n = std::min(int(batchEstimate.size()), int(keyframeLaserClouds.size()));
    for (int node_idx = 0; node_idx < n; node_idx++) {
        const gtsam::Pose3& pose = batchEstimate.at<gtsam::Pose3>(node_idx);
        Pose6D p6;
        p6.x = pose.translation().x();
        p6.y = pose.translation().y();
        p6.z = pose.translation().z();
        p6.roll = pose.rotation().roll();
        p6.pitch = pose.rotation().pitch();
        p6.yaw = pose.rotation().yaw();
        p6.seq = node_idx;
        *batchMap += *local2global(keyframeLaserClouds[node_idx], p6);
    }
    mKF.unlock();

    downSizeFilterMapSave.setInputCloud(batchMap);
    downSizeFilterMapSave.filter(*batchMap);

    // Re-express in the gravity-aligned/floor-referenced level frame (see the
    // level_frame parameter). Published once by the *_map_odom_bridge, so it
    // is available as soon as that bridge has leveled.
    std::string saved_frame = map_frame;
    if (save_in_level_frame) {
        // NO TIMEOUT. This used to pass tf2::durationFromSec(2.0), which HUNG
        // THE SERVICE FOREVER and cost a full mapping run:
        //
        //   * canTransform's timeout is counted in ROS TIME. Under
        //     use_sim_time the clock comes from the bag, and by the time anyone
        //     calls /pgo_batch_optimize the bag has FINISHED -- so /clock has
        //     stopped, ROS time is frozen, and a deadline of "now + 2 s" is
        //     never reached. The wait loop nanosleeps forever; the node sits at
        //     0% CPU looking deadlocked. The catch below cannot help, because
        //     it only runs if the timeout actually fires.
        //   * Separately, waiting on TF with a timeout from inside a callback
        //     on a SingleThreadedExecutor (which is what main() spins) is a
        //     deadlock pattern in its own right: the executor that would
        //     deliver /tf is the one being blocked.
        //
        // Waiting was never needed anyway. level_frame <- map_frame is STATIC,
        // published once by the *_map_odom_bridge early in the run, so it is
        // already in the buffer -- or it never will be, in which case failing
        // immediately into the fallback below is exactly right.
        std::string tf_err;
        if (!g_tf_buffer->canTransform(level_frame, map_frame,
                                       tf2::TimePointZero, &tf_err)) {
            RCLCPP_WARN(g_node->get_logger(),
                "No %s <- %s yet (%s). Saving in the RAW %s frame; see the note "
                "below on what that costs.",
                level_frame.c_str(), map_frame.c_str(), tf_err.c_str(),
                map_frame.c_str());
        }
        try {
            auto tf = g_tf_buffer->lookupTransform(
                level_frame, map_frame, tf2::TimePointZero);
            pcl::PointCloud<PointType>::Ptr leveled(new pcl::PointCloud<PointType>());
            pcl_ros::transformPointCloud(*batchMap, *leveled, tf);
            batchMap = leveled;
            saved_frame = level_frame;
            RCLCPP_INFO(g_node->get_logger(),
                "Saved map transformed %s -> %s (gravity-aligned, floor-referenced).",
                map_frame.c_str(), level_frame.c_str());
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(g_node->get_logger(),
                "Could not look up %s <- %s (%s). Saving in the RAW %s frame, "
                "which is NOT gravity-aligned -- a localization run against it "
                "will produce a tilted map frame that disagrees with any 2D grid "
                "built in %s.",
                level_frame.c_str(), map_frame.c_str(), ex.what(),
                map_frame.c_str(), level_frame.c_str());
        }
    }
    batchMap->header.frame_id = saved_frame;
    pcl::io::savePCDFileBinary(save_directory + "map_batch.pcd", *batchMap);

    std::ostringstream msg;
    msg << "Batch LM re-optimization done over " << n << " keyframes. "
        << "Graph error " << initialError << " -> " << finalError << ". "
        << "Poses: " << save_directory << "optimized_poses_batch.txt, "
        << "map: " << save_directory << "map_batch.pcd";
    response->success = true;
    response->message = msg.str();
    RCLCPP_INFO(g_node->get_logger(), "%s", msg.str().c_str());
}

void process_viz_map(void)
{
    rclcpp::Rate rate(vizmapFrequency);
    while (rclcpp::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubMap();
        }
    }
} // pointcloud_viz


int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	g_node = std::make_shared<rclcpp::Node>("laserPGO");
	g_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(g_node);
	g_tf_buffer = std::make_shared<tf2_ros::Buffer>(g_node->get_clock());
	g_tf_listener = std::make_shared<tf2_ros::TransformListener>(*g_tf_buffer);

    // LOCAL FIX: default was "/". Combined with the rm -r below that meant a
    // bare `ros2 run` (no launch file to override it) would try to delete
    // /Scans/ and write pose logs into the filesystem root. Defaulting under
    // the user's cache directory makes the unconfigured case harmless.
    {
        const char *home = std::getenv("HOME");
        std::string default_save = std::string(home ? home : "/tmp") + "/.cache/fastlio_lc_pgo/";
        g_node->declare_parameter<std::string>("save_directory", default_save);
    }
    save_directory = g_node->get_parameter("save_directory").as_string();
    if (save_directory.empty() || save_directory == "/") {
        RCLCPP_ERROR(g_node->get_logger(),
            "save_directory '%s' refuses to be used: this node deletes "
            "<save_directory>/Scans recursively at startup.", save_directory.c_str());
        return 1;
    }
    if (save_directory.back() != '/') save_directory += '/';

    // World frame for all published outputs (see map_frame declaration above).
    g_node->declare_parameter<std::string>("map_frame", "camera_init");
    map_frame = g_node->get_parameter("map_frame").as_string();

    // The saved map is a LOCALIZATION PRIOR: whatever frame it is written in
    // becomes the "map" frame of every downstream localization run. The raw
    // PGO map frame is anchored to the LIO start pose, which on a tilted
    // sensor mount is ~90 deg off gravity -- while the 2D occupancy grid built
    // alongside it (octomap, frame_id map_level) is gravity-aligned and
    // floor-referenced. Serving both as "map" then puts them ~90 deg apart.
    // Saving in level_frame makes the .pcd and the 2D grid share one frame.
    g_node->declare_parameter<std::string>("level_frame", "map_level");
    level_frame = g_node->get_parameter("level_frame").as_string();
    g_node->declare_parameter<bool>("save_in_level_frame", true);
    save_in_level_frame = g_node->get_parameter("save_in_level_frame").as_bool();
    pgKITTIformat = save_directory + "optimized_poses.txt";
    odomKITTIformat = save_directory + "odom_poses.txt";
    pgScansDirectory = save_directory + "Scans/";
    // create the save directory (and wipe/recreate its Scans/ subfolder) before
    // opening any output streams under it - times.txt lives directly in
    // save_directory, so this must happen first or the open silently fails.
    // LOCAL FIX: was an unquoted `system("exec rm -r " + pgScansDirectory)`, so
    // any space or shell metacharacter in save_directory changed what got
    // deleted. std::filesystem does the same job without a shell.
    {
        std::error_code ec;
        std::filesystem::remove_all(pgScansDirectory, ec);
        std::filesystem::create_directories(pgScansDirectory, ec);
        if (ec) {
            RCLCPP_ERROR(g_node->get_logger(), "Cannot create %s: %s",
                         pgScansDirectory.c_str(), ec.message().c_str());
            return 1;
        }
    }
    pgTimeSaveStream = std::fstream(save_directory + "times.txt", std::fstream::out);
    pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);

    g_node->declare_parameter<double>("keyframe_meter_gap", 2.0);
    keyframeMeterGap = g_node->get_parameter("keyframe_meter_gap").as_double();
    g_node->declare_parameter<double>("keyframe_deg_gap", 10.0);
    keyframeDegGap = g_node->get_parameter("keyframe_deg_gap").as_double();
    keyframeRadGap = deg2rad(keyframeDegGap);

    g_node->declare_parameter<double>("sc_dist_thres", 0.2);
    scDistThres = g_node->get_parameter("sc_dist_thres").as_double();
    g_node->declare_parameter<double>("sc_max_radius", 80.0);
    scMaximumRadius = g_node->get_parameter("sc_max_radius").as_double();
    // Height of the sensor above the floor, in the frame the keyframe clouds
    // arrive in. Upstream hardcoded 2.0 inside SCManager; on Pepper the L2 sits
    // at 0.2582 m, so that shifted every point ~1.75 m up before binning.
    g_node->declare_parameter<double>("sc_lidar_height", 2.0);
    scLidarHeight = g_node->get_parameter("sc_lidar_height").as_double();
    // Off by default: performSCLoopClosure() was dead code, so enabling it
    // changes detection behaviour and should be an explicit opt-in.
    g_node->declare_parameter<bool>("use_scan_context", false);
    useScanContext = g_node->get_parameter("use_scan_context").as_bool();

    // for loop closure detection
    g_node->declare_parameter<double>("historyKeyframeSearchRadius", 10.0);
    historyKeyframeSearchRadius = g_node->get_parameter("historyKeyframeSearchRadius").as_double();
    g_node->declare_parameter<double>("historyKeyframeSearchTimeDiff", 30.0);
    historyKeyframeSearchTimeDiff = g_node->get_parameter("historyKeyframeSearchTimeDiff").as_double();
    g_node->declare_parameter<int>("historyKeyframeSearchNum", 25);
    historyKeyframeSearchNum = g_node->get_parameter("historyKeyframeSearchNum").as_int();
    // loopNoiseScore is retained for backward compatibility: it is the fallback
    // for whichever of the split pair is left <= 0. See initNoises() for why one
    // scalar across all six DOF cannot be right when odomNoise splits them 100x.
    g_node->declare_parameter<double>("loopNoiseScore", 0.5);
    loopNoiseScore = g_node->get_parameter("loopNoiseScore").as_double();
    g_node->declare_parameter<double>("loopNoiseScoreRot", -1.0);
    loopNoiseScoreRot = g_node->get_parameter("loopNoiseScoreRot").as_double();
    g_node->declare_parameter<double>("loopNoiseScoreTrans", -1.0);
    loopNoiseScoreTrans = g_node->get_parameter("loopNoiseScoreTrans").as_double();
    if (loopNoiseScoreRot <= 0.0)   loopNoiseScoreRot = loopNoiseScore;
    if (loopNoiseScoreTrans <= 0.0) loopNoiseScoreTrans = loopNoiseScore;
    RCLCPP_INFO(g_node->get_logger(),
        "Loop factor noise: rotation %.2e rad^2, translation %.2e m^2 "
        "(odometry chain is 1e-6 / 1e-4 per edge)",
        loopNoiseScoreRot, loopNoiseScoreTrans);
    g_node->declare_parameter<int>("graphUpdateTimes", 2);
    graphUpdateTimes = g_node->get_parameter("graphUpdateTimes").as_int();
    g_node->declare_parameter<double>("loopFitnessScoreThreshold", 0.3);
    loopFitnessScoreThreshold = g_node->get_parameter("loopFitnessScoreThreshold").as_double();

    g_node->declare_parameter<double>("speedFactor", 1.0);
    speedFactor = g_node->get_parameter("speedFactor").as_double();
    {
        g_node->declare_parameter<double>("loopClosureFrequency", 2.0);
        loopClosureFrequency = g_node->get_parameter("loopClosureFrequency").as_double();
        loopClosureFrequency *= speedFactor;
        g_node->declare_parameter<double>("graphUpdateFrequency", 1.0);
        graphUpdateFrequency = g_node->get_parameter("graphUpdateFrequency").as_double();
        graphUpdateFrequency *= speedFactor;
        g_node->declare_parameter<double>("vizmapFrequency", 0.1);
        vizmapFrequency = g_node->get_parameter("vizmapFrequency").as_double();
        vizmapFrequency *= speedFactor;
        g_node->declare_parameter<double>("vizPathFrequency", 10.0);
        vizPathFrequency = g_node->get_parameter("vizPathFrequency").as_double();
        vizPathFrequency *= speedFactor;
    }

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    initNoises();

    scManager.setSCdistThres(scDistThres);
    scManager.setMaximumRadius(scMaximumRadius);
    scManager.setLidarHeight(scLidarHeight);
    RCLCPP_INFO(g_node->get_logger(),
        "Scan Context place recognition: %s (dist_thres %.2f, max_radius %.1f m, lidar_height %.3f m)",
        useScanContext ? "ENABLED" : "disabled (descriptors still built)",
        scDistThres, scMaximumRadius, scLidarHeight);

    // LOCAL FIX: this leaf size was hardcoded at 0.4 m, and it is applied to
    // every keyframe BEFORE the cloud is stored in keyframeLaserClouds (see
    // process_pg). Everything downstream -- the RViz map, map_batch.pcd, the
    // ICP submaps -- is therefore built from 0.4 m data, which is why asking
    // for map_save_filter_size 0.05 could not produce a dense localization
    // prior: the resolution was already gone. FAST-LIO's own filter_size_surf
    // (0.25 on the L2) is the real floor, so values below that gain nothing.
    // Kept at 0.4 by default to preserve existing behaviour.
    g_node->declare_parameter<double>("keyframe_filter_size", 0.4);
    float filter_size = float(g_node->get_parameter("keyframe_filter_size").as_double());
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    g_node->declare_parameter<double>("mapviz_filter_size", 0.4);
    double mapVizFilterSize = g_node->get_parameter("mapviz_filter_size").as_double();
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

    // <= 0 means "same as mapviz_filter_size" (previous behaviour, one shared
    // filter). Set it smaller than mapviz_filter_size to write a dense
    // localization prior while keeping the RViz map cheap.
    g_node->declare_parameter<double>("map_save_filter_size", -1.0);
    double mapSaveFilterSize = g_node->get_parameter("map_save_filter_size").as_double();
    if (mapSaveFilterSize <= 0.0) mapSaveFilterSize = mapVizFilterSize;
    downSizeFilterMapSave.setLeafSize(mapSaveFilterSize, mapSaveFilterSize, mapSaveFilterSize);
    RCLCPP_INFO(g_node->get_logger(),
        "PGO map leaf sizes: rviz %.3f m, saved map_batch.pcd %.3f m",
        mapVizFilterSize, mapSaveFilterSize);

    g_node->declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
    g_node->declare_parameter<std::string>("odom_topic", "/Odometry");
    std::string cloud_topic = g_node->get_parameter("cloud_topic").as_string();
    std::string odom_topic = g_node->get_parameter("odom_topic").as_string();

	auto subLaserCloudFullRes = g_node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::QoS(100), laserCloudFullResHandler);
	auto subLaserOdometry = g_node->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS(100), laserOdometryHandler);
	auto subGPS = g_node->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", rclcpp::QoS(100), gpsHandler);

	pubOdomAftPGO = g_node->create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", 100);
	pubOdomRepubVerifier = g_node->create_publisher<nav_msgs::msg::Odometry>("/repub_odom", 100);

    // for front-end
    pubKeyFramesId = g_node->create_publisher<std_msgs::msg::Header>("/key_frames_ids", 10);

    // for loop closure
    pubLoopConstraintEdge = g_node->create_publisher<visualization_msgs::msg::MarkerArray>("/loop_closure_constraints", 1);
	pubLoopScanLocalRegisted = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local_registed", 100);

	pubPathAftPGO = g_node->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", 100);
	pubMapAftPGO = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("/aft_pgo_map", 100);

	pubLoopScanLocal = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", 100);
	pubLoopSubmapLocal = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", 100);

	srvBatchOptimize = g_node->create_service<std_srvs::srv::Trigger>(
        "/pgo_batch_optimize", batchOptimizeHandler);

	std::thread posegraph_slam {process_pg}; // pose graph construction
	std::thread lc_detection {process_lcd}; // loop closure detection
	std::thread icp_calculation {process_icp}; // loop constraint calculation via icp
	std::thread isam_update {process_isam}; // isam2 optimization

	std::thread viz_map {process_viz_map}; // visualization - map (low frequency because it is heavy)

 	rclcpp::spin(g_node);
 	rclcpp::shutdown();

	// LOCAL FIX: join before returning. Every one of these threads loops on
	// rclcpp::ok(), so shutdown() above ends them -- but without the joins their
	// destructors ran while still joinable, which is std::terminate(), i.e. the
	// node aborted on every clean Ctrl-C. Flush the time log too; it was never
	// closed explicitly.
	posegraph_slam.join();
	lc_detection.join();
	icp_calculation.join();
	isam_update.join();
	viz_map.join();

	if (pgTimeSaveStream.is_open()) {
		pgTimeSaveStream.flush();
		pgTimeSaveStream.close();
	}

	return 0;
}
