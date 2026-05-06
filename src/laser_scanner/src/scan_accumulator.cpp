#include <algorithm>
#include <memory>
#include <numeric>
#include <vector>
#include <mutex>
#include <filesystem>
#include <cmath>
#include <omp.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

// small_gicp headers (native API)
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/registration/registration_helper.hpp>

namespace fs = std::filesystem;

// ============================================================================
// CONFIGURATION
// ============================================================================
static const int    AUTO_PUBLISH_INTERVAL_MS = 1000;

// GICP — run on structure-only subset (non-floor points) so the solver
// locks onto real 3D geometry rather than featureless flat floor.
static const double GICP_MAX_CORRESPONDENCE_DISTANCE = 0.25;  // 250 mm overlapping strips
static const int    GICP_MAX_ITERATIONS              = 200;
static const double GICP_DOWNSAMPLING_RESOLUTION     = 0.005; // 5 mm voxel (more points survive)
static const int    GICP_NUM_THREADS                 = 4;
static const int    GICP_NUM_NEIGHBORS               = 20;
static const double OVERLAP_ROI_RATIO                = 0.30;  // use edge bands only for strip-to-strip GICP
static const int    DYNAMIC_ROI_BINS                 = 24;
static const double DYNAMIC_ROI_SCORE_THRESHOLD      = 0.45;
static const int    DYNAMIC_ROI_MIN_BIN_POINTS       = 1500;

// Floor filtering for GICP input
// Points whose Z (in base_link) is within FLOOR_BAND_M of the detected
// floor level are excluded from registration.  Full-resolution merged
// cloud always keeps all points.
// NOTE: floor_z is detected from scan #1 and reused for all subsequent scans
//       to ensure consistent structure extraction across all strips.
static const double FLOOR_BAND_M            = 0.020;  // +-20 mm around floor = removed
static const double FLOOR_DETECT_PERCENTILE = 0.02;   // bottom 2% of Z = floor level
// ============================================================================

static std::string getStringParameter(
    rclcpp::Node& node,
    const std::string& name,
    const std::string& default_value)
{
    const auto param = node.get_parameter(name);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        return param.as_string();
    }

    if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        // YAML 1.1 treats bare "y" as true and bare "n" as false.
        // Launch files can hit this even when the intended parameter is a string.
        return param.as_bool() ? "y" : "n";
    }

    return default_value;
}

// ---------------------------------------------------------------------------
// Detect floor Z level as the P-th percentile of point Z values
// ---------------------------------------------------------------------------
static float detectFloorZ(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                           double percentile)
{
    std::vector<float> zvals;
    zvals.reserve(cloud->size());
    for (const auto& p : cloud->points) zvals.push_back(p.z);
    size_t idx = static_cast<size_t>(percentile * zvals.size());
    idx = std::min(idx, zvals.size() - 1);
    std::nth_element(zvals.begin(), zvals.begin() + idx, zvals.end());
    return zvals[idx];
}

// ---------------------------------------------------------------------------
// Remove floor band and return a structure-only cloud for GICP
// ---------------------------------------------------------------------------
static pcl::PointCloud<pcl::PointXYZI>::Ptr removeFloor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    float floor_z, float band)
{
    auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    out->reserve(cloud->size() / 4);
    for (const auto& p : cloud->points) {
        if (std::abs(p.z - floor_z) > band)
            out->points.push_back(p);
    }
    out->width = out->points.size();
    out->height = 1;
    out->is_dense = true;
    return out;
}

// ---------------------------------------------------------------------------
// Keep only the left or right X-edge band of a cloud.
// This lets GICP focus on the actual strip overlap instead of the full strip.
// ---------------------------------------------------------------------------
static pcl::PointCloud<pcl::PointXYZI>::Ptr cropAxisEdge(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    int axis_idx, double ratio, bool keep_high)
{
    auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (cloud->empty()) {
        out->width = 0;
        out->height = 1;
        out->is_dense = true;
        return out;
    }

    ratio = std::clamp(ratio, 0.05, 0.95);

    float axis_min = std::numeric_limits<float>::max();
    float axis_max = std::numeric_limits<float>::lowest();
    for (const auto& p : cloud->points) {
        const float coord = axis_idx == 0 ? p.x : p.y;
        axis_min = std::min(axis_min, coord);
        axis_max = std::max(axis_max, coord);
    }

    const float span = axis_max - axis_min;
    const float band = static_cast<float>(span * ratio);
    const float threshold = keep_high ? (axis_max - band) : (axis_min + band);

    out->reserve(cloud->size() / 2);
    for (const auto& p : cloud->points) {
        const float coord = axis_idx == 0 ? p.x : p.y;
        if (keep_high) {
            if (coord >= threshold) out->points.push_back(p);
        } else {
            if (coord <= threshold) out->points.push_back(p);
        }
    }
    out->width = out->points.size();
    out->height = 1;
    out->is_dense = true;
    return out;
}

struct RoiSelection {
    pcl::PointCloud<pcl::PointXYZI>::Ptr target;
    pcl::PointCloud<pcl::PointXYZI>::Ptr source;
    std::string info;
    bool valid;
};

static RoiSelection buildDynamicRoi(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
    int axis_idx,
    int bins,
    double score_threshold,
    int min_bin_points)
{
    RoiSelection out;
    out.target = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    out.source = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    out.valid = false;

    if (target->empty() || source->empty()) {
        out.info = "empty structure clouds";
        return out;
    }

    float target_min = std::numeric_limits<float>::max();
    float target_max = std::numeric_limits<float>::lowest();
    for (const auto& p : target->points) {
        const float coord = axis_idx == 0 ? p.x : p.y;
        target_min = std::min(target_min, coord);
        target_max = std::max(target_max, coord);
    }

    float source_min = std::numeric_limits<float>::max();
    float source_max = std::numeric_limits<float>::lowest();
    for (const auto& p : source->points) {
        const float coord = axis_idx == 0 ? p.x : p.y;
        source_min = std::min(source_min, coord);
        source_max = std::max(source_max, coord);
    }

    const float overlap_lo = std::max(target_min, source_min);
    const float overlap_hi = std::min(target_max, source_max);
    if (overlap_hi <= overlap_lo) {
        out.info = "no axis overlap";
        return out;
    }

    bins = std::max(4, bins);
    const double step = static_cast<double>(overlap_hi - overlap_lo) / static_cast<double>(bins);
    if (step <= 0.0) {
        out.info = "degenerate overlap span";
        return out;
    }

    std::vector<double> scores(bins, 0.0);
    double max_score = 0.0;
    int best_start = -1;
    int best_end = -1;
    double best_sum = -1.0;

    for (int i = 0; i < bins; ++i) {
        const float bin_lo = static_cast<float>(overlap_lo + step * i);
        const float bin_hi = (i == bins - 1)
            ? overlap_hi
            : static_cast<float>(overlap_lo + step * (i + 1));

        std::vector<float> target_z;
        std::vector<float> source_z;
        target_z.reserve(4096);
        source_z.reserve(4096);

        for (const auto& p : target->points) {
            const float coord = axis_idx == 0 ? p.x : p.y;
            const bool in_bin = (i == bins - 1)
                ? (coord >= bin_lo && coord <= bin_hi)
                : (coord >= bin_lo && coord < bin_hi);
            if (in_bin) target_z.push_back(p.z);
        }
        for (const auto& p : source->points) {
            const float coord = axis_idx == 0 ? p.x : p.y;
            const bool in_bin = (i == bins - 1)
                ? (coord >= bin_lo && coord <= bin_hi)
                : (coord >= bin_lo && coord < bin_hi);
            if (in_bin) source_z.push_back(p.z);
        }

        if (static_cast<int>(target_z.size()) < min_bin_points ||
            static_cast<int>(source_z.size()) < min_bin_points)
        {
            continue;
        }

        auto calc_stats = [](const std::vector<float>& zvals, double& z_std, double& z_range) {
            const double mean = std::accumulate(zvals.begin(), zvals.end(), 0.0) / static_cast<double>(zvals.size());
            double accum = 0.0;
            float z_min = std::numeric_limits<float>::max();
            float z_max = std::numeric_limits<float>::lowest();
            for (float z : zvals) {
                const double dz = static_cast<double>(z) - mean;
                accum += dz * dz;
                z_min = std::min(z_min, z);
                z_max = std::max(z_max, z);
            }
            z_std = std::sqrt(accum / static_cast<double>(zvals.size()));
            z_range = static_cast<double>(z_max - z_min);
        };

        double target_std = 0.0, target_range = 0.0;
        double source_std = 0.0, source_range = 0.0;
        calc_stats(target_z, target_std, target_range);
        calc_stats(source_z, source_std, source_range);

        const double z_std = std::min(target_std, source_std);
        const double z_range = std::min(target_range, source_range);
        const double count_term = std::log1p(static_cast<double>(std::min(target_z.size(), source_z.size())));
        scores[i] = count_term * (0.7 * z_std + 0.3 * z_range);
        max_score = std::max(max_score, scores[i]);
    }

    if (max_score <= 0.0) {
        out.info = "no structured overlap";
        return out;
    }

    int run_start = -1;
    double run_sum = 0.0;
    for (int i = 0; i < bins; ++i) {
        const bool good = scores[i] >= max_score * score_threshold;
        if (good) {
            if (run_start < 0) {
                run_start = i;
                run_sum = 0.0;
            }
            run_sum += scores[i];
        } else if (run_start >= 0) {
            const int run_end = i - 1;
            if (run_sum > best_sum) {
                best_sum = run_sum;
                best_start = run_start;
                best_end = run_end;
            }
            run_start = -1;
            run_sum = 0.0;
        }
    }
    if (run_start >= 0) {
        const int run_end = bins - 1;
        if (run_sum > best_sum) {
            best_sum = run_sum;
            best_start = run_start;
            best_end = run_end;
        }
    }

    if (best_start < 0) {
        best_start = static_cast<int>(std::distance(scores.begin(), std::max_element(scores.begin(), scores.end())));
        best_end = best_start;
    }

    const float roi_lo = static_cast<float>(overlap_lo + step * best_start);
    const float roi_hi = (best_end == bins - 1)
        ? overlap_hi
        : static_cast<float>(overlap_lo + step * (best_end + 1));

    out.target->reserve(target->size() / 4);
    out.source->reserve(source->size() / 4);
    for (const auto& p : target->points) {
        const float coord = axis_idx == 0 ? p.x : p.y;
        if (coord >= roi_lo && coord <= roi_hi) out.target->points.push_back(p);
    }
    for (const auto& p : source->points) {
        const float coord = axis_idx == 0 ? p.x : p.y;
        if (coord >= roi_lo && coord <= roi_hi) out.source->points.push_back(p);
    }
    out.target->width = out.target->points.size();
    out.target->height = 1;
    out.target->is_dense = true;
    out.source->width = out.source->points.size();
    out.source->height = 1;
    out.source->is_dense = true;
    out.valid = !out.target->empty() && !out.source->empty();

    const char axis_name = axis_idx == 0 ? 'x' : 'y';
    out.info = std::string(1, axis_name) + "[" + std::to_string(roi_lo) + "," +
               std::to_string(roi_hi) + "] bins=" + std::to_string(best_start) + "-" +
               std::to_string(best_end) + " pts=" + std::to_string(out.target->size()) + "/" +
               std::to_string(out.source->size());
    return out;
}

class ScanAccumulatorNode : public rclcpp::Node
{
public:
    ScanAccumulatorNode() : Node("scan_accumulator"), scan_count_(0), block_scan_count_(0)
    {
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scanner/single_scan", 10,
            std::bind(&ScanAccumulatorNode::scanCallback, this, std::placeholders::_1));

        merged_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/scanner/merged_cloud",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/scanner/save_merged_cloud",
            std::bind(&ScanAccumulatorNode::saveMergedCloudCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        clear_service_ = this->create_service<std_srvs::srv::Empty>(
            "/scanner/clear_scans",
            std::bind(&ScanAccumulatorNode::clearScansCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        block_ready_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/scanner/block_ready",
            std::bind(&ScanAccumulatorNode::blockReadyCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        count_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/scanner/get_scan_count",
            std::bind(&ScanAccumulatorNode::getScanCountCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Parameters
        this->declare_parameter("fixed_frame", "base_link");
        this->declare_parameter("output_directory",
                                std::string(getenv("HOME")) + "/Dobot_Robot_Arm/scans");
        this->declare_parameter("position_prefix", std::string("pos1"));  // change between car positions
        this->declare_parameter("auto_publish_interval_ms", AUTO_PUBLISH_INTERVAL_MS);
        this->declare_parameter("gicp_max_correspondence_distance", GICP_MAX_CORRESPONDENCE_DISTANCE);
        this->declare_parameter("gicp_max_iterations",              GICP_MAX_ITERATIONS);
        this->declare_parameter("gicp_downsampling_resolution",     GICP_DOWNSAMPLING_RESOLUTION);
        this->declare_parameter("gicp_num_threads",                 GICP_NUM_THREADS);
        this->declare_parameter("overlap_roi_mode",                 std::string("dynamic_z"));
        rcl_interfaces::msg::ParameterDescriptor axis_descriptor;
        axis_descriptor.dynamic_typing = true;
        this->declare_parameter("overlap_roi_axis",
                                rclcpp::ParameterValue(std::string("y")),
                                axis_descriptor);
        this->declare_parameter("overlap_roi_ratio",                OVERLAP_ROI_RATIO);
        this->declare_parameter("dynamic_roi_bins",                 DYNAMIC_ROI_BINS);
        this->declare_parameter("dynamic_roi_score_threshold",      DYNAMIC_ROI_SCORE_THRESHOLD);
        this->declare_parameter("dynamic_roi_min_bin_points",       DYNAMIC_ROI_MIN_BIN_POINTS);
        this->declare_parameter("floor_band_m",                     FLOOR_BAND_M);

        fixed_frame_       = this->get_parameter("fixed_frame").as_string();
        output_directory_  = this->get_parameter("output_directory").as_string();
        position_prefix_   = this->get_parameter("position_prefix").as_string();
        gicp_max_corr_    = this->get_parameter("gicp_max_correspondence_distance").as_double();
        gicp_max_iter_    = this->get_parameter("gicp_max_iterations").as_int();
        gicp_downsample_  = this->get_parameter("gicp_downsampling_resolution").as_double();
        gicp_num_threads_ = this->get_parameter("gicp_num_threads").as_int();
        overlap_roi_mode_ = this->get_parameter("overlap_roi_mode").as_string();
        if (overlap_roi_mode_ != "dynamic_z" &&
            overlap_roi_mode_ != "fixed" &&
            overlap_roi_mode_ != "tf_only")
        {
            RCLCPP_WARN(this->get_logger(),
                "[ACCUMULATOR] Unknown overlap_roi_mode='%s'; defaulting to dynamic_z.",
                overlap_roi_mode_.c_str());
            overlap_roi_mode_ = "dynamic_z";
        }
        overlap_roi_axis_ = getStringParameter(*this, "overlap_roi_axis", "y");
        if (overlap_roi_axis_ == "n") {
            RCLCPP_WARN(this->get_logger(),
                "[ACCUMULATOR] overlap_roi_axis was parsed as boolean false; defaulting to y.");
            overlap_roi_axis_ = "y";
        }
        overlap_roi_ratio_ = this->get_parameter("overlap_roi_ratio").as_double();
        dynamic_roi_bins_ = this->get_parameter("dynamic_roi_bins").as_int();
        dynamic_roi_score_threshold_ = this->get_parameter("dynamic_roi_score_threshold").as_double();
        dynamic_roi_min_bin_points_ = this->get_parameter("dynamic_roi_min_bin_points").as_int();
        floor_band_       = this->get_parameter("floor_band_m").as_double();

        try {
            fs::create_directories(output_directory_);
            RCLCPP_INFO(this->get_logger(), "Output directory: %s", output_directory_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create output dir: %s", e.what());
            output_directory_ = "/tmp";
        }

        int pub_ms = this->get_parameter("auto_publish_interval_ms").as_int();
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(pub_ms),
            std::bind(&ScanAccumulatorNode::publishMergedCloud, this));

        accumulated_cloud_    = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        reference_floor_z_    = 0.0f;
        floor_z_initialized_  = false;
        prev_tf_translation_  = Eigen::Vector3d::Zero();

        RCLCPP_INFO(this->get_logger(),
            "[ACCUMULATOR] Ready — structure-GICP + TF, fixed_frame=%s, prefix=%s, "
            "corr=%.3fm, roi_mode=%s, roi_axis=%s, overlap_roi=%.0f%%, floor_band=+-%.1fmm",
            fixed_frame_.c_str(), position_prefix_.c_str(), gicp_max_corr_,
            overlap_roi_mode_.c_str(), overlap_roi_axis_.c_str(),
            overlap_roi_ratio_ * 100.0, floor_band_ * 1000.0);
    }

private:

    // ---------------------------------------------------------------
    // Parse PointCloud2 -> pcl::PointXYZI  (filters NaN / zero-Z)
    // ---------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr msgToPcl(
        const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
    {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        int num_points = msg->width * msg->height;
        cloud->points.reserve(num_points);

        int x_off = -1, y_off = -1, z_off = -1, i_off = -1;
        for (const auto& f : msg->fields) {
            if (f.name == "x")         x_off = f.offset;
            if (f.name == "y")         y_off = f.offset;
            if (f.name == "z")         z_off = f.offset;
            if (f.name == "intensity") i_off = f.offset;
        }

        for (int idx = 0; idx < num_points; ++idx) {
            const uint8_t* ptr = &msg->data[idx * msg->point_step];
            float x = *reinterpret_cast<const float*>(ptr + x_off);
            float y = *reinterpret_cast<const float*>(ptr + y_off);
            float z = *reinterpret_cast<const float*>(ptr + z_off);

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)
                || z == 0.0f || z <= -0.999f)
                continue;

            pcl::PointXYZI pt;
            pt.x = x;  pt.y = y;  pt.z = z;
            float intensity = (i_off >= 0) ? *reinterpret_cast<const float*>(ptr + i_off) : 0.0f;
            pt.intensity = intensity;
            cloud->points.push_back(pt);
        }
        cloud->width  = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;
        return cloud;
    }

    // ---------------------------------------------------------------
    // PCL -> small_gicp::PointCloud (full 3D X,Y,Z)
    // ---------------------------------------------------------------
    std::shared_ptr<small_gicp::PointCloud> toSmallGicp3D(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& in)
    {
        auto sg = std::make_shared<small_gicp::PointCloud>();
        sg->resize(in->size());
        for (size_t i = 0; i < in->size(); ++i)
            sg->point(i) << in->points[i].x, in->points[i].y, in->points[i].z, 1.0;
        return sg;
    }

    // ---------------------------------------------------------------
    // TF lookup: laser_frame -> fixed_frame
    // ---------------------------------------------------------------
    bool lookupPose(const rclcpp::Time& stamp, Eigen::Isometry3d& pose_out)
    {
        try {
            auto tf = tf_buffer_->lookupTransform(
                fixed_frame_, "laser_frame", stamp,
                rclcpp::Duration::from_seconds(0.5));
            pose_out = tf2::transformToEigen(tf);
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            pose_out = Eigen::Isometry3d::Identity();
            return false;
        }
    }

    // ---------------------------------------------------------------
    // Main scan callback
    // ---------------------------------------------------------------
    void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        // 1. Parse PointCloud2 -> pcl::PointXYZI
        auto scan_pcl = msgToPcl(msg);
        if (scan_pcl->empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty scan -- skipping");
            return;
        }

        // 2. Robot arm FK: laser_frame -> base_link
        Eigen::Isometry3d tf_pose;
        bool have_tf = lookupPose(rclcpp::Time(msg->header.stamp), tf_pose);

        Eigen::Vector3d tf_t = tf_pose.translation();
        RCLCPP_INFO(this->get_logger(),
            "[ACCUMULATOR] Scan #%d: %zu pts  |  TF %s  laser->base: (%.3f, %.3f, %.3f) m",
            scan_count_ + 1, scan_pcl->size(),
            have_tf ? "OK" : "IDENTITY-FALLBACK",
            tf_t.x(), tf_t.y(), tf_t.z());

        // Use the full TF (including Y) — the TF Y translation IS the real strip
        // offset in world coordinates.  Each strip starts at a different Y position
        // because the arm returns to a new Y start between strips.
        Eigen::Matrix4f tf_matrix = tf_pose.matrix().cast<float>();

        // 3. Transform entire scan into base_link frame
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_in_fixed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud<pcl::PointXYZI>(*scan_pcl, *scan_in_fixed, tf_matrix);

        // 4. Detect floor Z from first scan and reuse for all subsequent scans.
        float floor_z;
        if (!floor_z_initialized_) {
            floor_z = detectFloorZ(scan_in_fixed, FLOOR_DETECT_PERCENTILE);
            reference_floor_z_   = floor_z;
            floor_z_initialized_ = true;
            RCLCPP_INFO(this->get_logger(),
                "[ACCUMULATOR] Floor reference established: floor_z=%.4f m (will reuse for all scans)",
                floor_z);
        } else {
            floor_z = reference_floor_z_;
        }

        // Report X, Y, Z extents
        float z_min = floor_z, z_max = floor_z;
        float x_min = std::numeric_limits<float>::max();
        float x_max = std::numeric_limits<float>::lowest();
        float y_min = std::numeric_limits<float>::max();
        float y_max = std::numeric_limits<float>::lowest();
        for (const auto& p : scan_in_fixed->points) {
            z_min = std::min(z_min, p.z);
            z_max = std::max(z_max, p.z);
            x_min = std::min(x_min, p.x);
            x_max = std::max(x_max, p.x);
            y_min = std::min(y_min, p.y);
            y_max = std::max(y_max, p.y);
        }
        RCLCPP_INFO(this->get_logger(),
            "[ACCUMULATOR] Scan #%d: floor_z=%.3fm  Z[%.3f..%.3f]  "
            "X[%.3f..%.3f](span=%.3fm)  Y[%.3f..%.3f](span=%.3fm)",
            scan_count_ + 1, floor_z, z_min, z_max,
            x_min, x_max, x_max - x_min,
            y_min, y_max, y_max - y_min);

        // Save individual scan PCD: blockN/blockN_scanM.pcd
        int block_num = current_block_index_;
        int scan_num  = block_scan_count_ + 1;
        std::string block_folder = output_directory_ + "/block" + std::to_string(block_num);
        try {
            fs::create_directories(block_folder);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create block folder: %s", e.what());
        }
        std::string individual_path = block_folder + "/block" + std::to_string(block_num) + "_scan" + std::to_string(scan_num) + ".pcd";
        try {
            pcl::io::savePCDFileBinary(individual_path, *scan_in_fixed);
            RCLCPP_INFO(this->get_logger(),
                "[ACCUMULATOR] Saved individual scan #%d -> %s (%zu pts)",
                scan_num, individual_path.c_str(), scan_in_fixed->size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save individual scan: %s", e.what());
        }

        // 5. First scan -- store directly, no registration needed
        if (scan_count_ == 0 || accumulated_cloud_->empty()) {
            *accumulated_cloud_ += *scan_in_fixed;
            RCLCPP_INFO(this->get_logger(),
                "[ACCUMULATOR] Scan #1 stored (TF %s) -> Total: %zu pts",
                have_tf ? "OK" : "IDENTITY-FALLBACK",
                accumulated_cloud_->size());
            finishScan(block_num, block_folder, tf_t);
            return;
        }

        if (overlap_roi_mode_ == "tf_only") {
            *accumulated_cloud_ += *scan_in_fixed;
            RCLCPP_INFO(this->get_logger(),
                "[ACCUMULATOR] Scan #%d: TF-only mode -> Total: %zu pts",
                scan_count_ + 1, accumulated_cloud_->size());
            finishScan(block_num, block_folder, tf_t);
            return;
        }

        // 6. GICP refinement on top of TF placement.
        //
        //    The full TF already placed scan_in_fixed in world space. Adjacent strips
        //    overlap by ~0.76m in Y, so GICP can find correspondences in that shared
        //    region and correct any small FK error. Init = identity (no extra offset
        //    needed — points are already in world coordinates).

        auto map_structure  = removeFloor(accumulated_cloud_, reference_floor_z_,
                                          static_cast<float>(floor_band_));
        auto scan_structure = removeFloor(scan_in_fixed, reference_floor_z_,
                                          static_cast<float>(floor_band_));

        if (map_structure->size() < 100 || scan_structure->size() < 100) {
            *accumulated_cloud_ += *scan_in_fixed;
            RCLCPP_WARN(this->get_logger(),
                "[ACCUMULATOR] Scan #%d: too few structure pts -- TF-only -> Total: %zu pts",
                scan_count_ + 1, accumulated_cloud_->size());
            finishScan(block_num, block_folder, tf_t);
            return;
        }

        const int axis_idx = (overlap_roi_axis_ == "x") ? 0 : 1;
        RoiSelection roi_selection;
        if (overlap_roi_mode_ == "dynamic_z") {
            roi_selection = buildDynamicRoi(
                map_structure, scan_structure, axis_idx,
                dynamic_roi_bins_, dynamic_roi_score_threshold_, dynamic_roi_min_bin_points_);
        } else if (overlap_roi_mode_ == "fixed") {
            roi_selection.target = cropAxisEdge(map_structure, axis_idx, overlap_roi_ratio_, true);
            roi_selection.source = cropAxisEdge(scan_structure, axis_idx, overlap_roi_ratio_, false);
            roi_selection.info = overlap_roi_axis_ + std::string("-fixed ") +
                                 std::to_string(static_cast<int>(overlap_roi_ratio_ * 100.0)) + "%";
            roi_selection.valid = !roi_selection.target->empty() && !roi_selection.source->empty();
        }

        auto map_roi  = roi_selection.target;
        auto scan_roi = roi_selection.source;

        if (!roi_selection.valid || map_roi->size() < 100 || scan_roi->size() < 100) {
            *accumulated_cloud_ += *scan_in_fixed;
            RCLCPP_WARN(this->get_logger(),
                "[ACCUMULATOR] Scan #%d: overlap ROI unavailable (%s, map=%zu scan=%zu) -- TF-only -> Total: %zu pts",
                scan_count_ + 1, roi_selection.info.c_str(), map_roi->size(), scan_roi->size(),
                accumulated_cloud_->size());
            finishScan(block_num, block_folder, tf_t);
            return;
        }

        auto sg_target = toSmallGicp3D(map_roi);
        auto sg_source = toSmallGicp3D(scan_roi);

        auto [target_ds, target_tree] = small_gicp::preprocess_points(
            *sg_target, gicp_downsample_, GICP_NUM_NEIGHBORS, gicp_num_threads_);
        auto [source_ds, source_tree] = small_gicp::preprocess_points(
            *sg_source, gicp_downsample_, GICP_NUM_NEIGHBORS, gicp_num_threads_);

        RCLCPP_INFO(this->get_logger(),
            "[ACCUMULATOR] GICP #%d ROI: %s  map=%zu  scan=%zu  map_ds=%zu  scan_ds=%zu  corr=%.3fm",
            scan_count_ + 1, roi_selection.info.c_str(), map_roi->size(), scan_roi->size(),
            target_ds->size(), source_ds->size(), gicp_max_corr_);

        small_gicp::RegistrationSetting setting;
        setting.type = small_gicp::RegistrationSetting::GICP;
        setting.max_correspondence_distance = gicp_max_corr_;
        setting.max_iterations = gicp_max_iter_;
        setting.num_threads = gicp_num_threads_;

        // Identity init — points are already in world space via TF
        Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
        auto result = small_gicp::align(*target_ds, *source_ds, *target_tree, init_T, setting);

        Eigen::Vector3d dt = result.T_target_source.translation();
        Eigen::AngleAxisd aa(result.T_target_source.rotation());
        double dt_m   = dt.norm();
        double dr_deg = aa.angle() * 180.0 / M_PI;

        // Allow up to 150mm / 10deg correction on top of TF placement
        constexpr double MAX_GICP_TRANSLATION_M = 0.08;
        constexpr double MAX_GICP_ROTATION_DEG  = 5.0;

        if (result.converged && result.iterations > 0 &&
            dt_m < MAX_GICP_TRANSLATION_M && dr_deg < MAX_GICP_ROTATION_DEG)
        {
            Eigen::Matrix4f correction = result.T_target_source.matrix().cast<float>();
            pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud<pcl::PointXYZI>(*scan_in_fixed, *aligned, correction);
            *accumulated_cloud_ += *aligned;
            RCLCPP_INFO(this->get_logger(),
                "[ACCUMULATOR] Scan #%d: GICP OK  iters=%zu  dt=%.4fm  dr=%.3fdeg -> Total: %zu pts",
                scan_count_ + 1, result.iterations, dt_m, dr_deg, accumulated_cloud_->size());
        } else {
            // TF-only fallback
            *accumulated_cloud_ += *scan_in_fixed;
            RCLCPP_WARN(this->get_logger(),
                "[ACCUMULATOR] Scan #%d: GICP %s (iters=%zu dt=%.4fm dr=%.2fdeg) -- TF-only -> Total: %zu pts",
                scan_count_ + 1,
                (!result.converged) ? "FAIL" : (result.iterations == 0) ? "NO-OVERLAP" : "REJECTED",
                result.iterations, dt_m, dr_deg, accumulated_cloud_->size());
        }

        finishScan(block_num, block_folder, tf_t);
    }

    void finishScan(int block_num, const std::string& block_folder, const Eigen::Vector3d& tf_t)
    {
        prev_tf_translation_ = tf_t;
        scan_count_++;
        block_scan_count_++;

        // After accumulation, if scan_num == 3, save merged block and clear
        if (block_scan_count_ == 3) {
            // Debug: check intensity range before saving
            float min_intensity = std::numeric_limits<float>::max();
            float max_intensity = std::numeric_limits<float>::lowest();
            for (const auto& p : accumulated_cloud_->points) {
                min_intensity = std::min(min_intensity, p.intensity);
                max_intensity = std::max(max_intensity, p.intensity);
            }
            RCLCPP_INFO(this->get_logger(),
                "[ACCUMULATOR] Saving block #%d: intensity range [%.3f .. %.3f] (type: PointXYZI)",
                block_num, min_intensity, max_intensity);
            std::string merged_path = block_folder + "/block" + std::to_string(block_num) + ".pcd";
            try {
                pcl::io::savePCDFileBinary(merged_path, *accumulated_cloud_);
                RCLCPP_INFO(this->get_logger(),
                    "[ACCUMULATOR] Saved merged block #%d -> %s (%zu pts)",
                    block_num, merged_path.c_str(), accumulated_cloud_->size());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to save merged block: %s", e.what());
            }

            // Push the completed block immediately so map_merger has a fresh
            // /scanner/merged_cloud before we clear local state.
            sensor_msgs::msg::PointCloud2 out;
            pcl::toROSMsg(*accumulated_cloud_, out);
            out.header.frame_id = fixed_frame_;
            out.header.stamp = this->get_clock()->now();
            merged_publisher_->publish(out);
            completed_block_ready_ = true;
            completed_block_points_ = accumulated_cloud_->size();
            RCLCPP_INFO(this->get_logger(),
                "[ACCUMULATOR] Published completed block #%d on /scanner/merged_cloud",
                block_num);

            accumulated_cloud_->clear(); // Start new block
            scan_count_ = 0;
            block_scan_count_ = 0;
            current_block_index_++;
        }
    }

    // ---------------------------------------------------------------
    // Periodic publish for RViz
    // ---------------------------------------------------------------
    void publishMergedCloud()
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (accumulated_cloud_->empty()) return;

        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(*accumulated_cloud_, out);
        out.header.frame_id = fixed_frame_;
        out.header.stamp    = this->get_clock()->now();
        merged_publisher_->publish(out);
    }

    // ---------------------------------------------------------------
    // Save
    // ---------------------------------------------------------------
    void saveMergedCloudCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (accumulated_cloud_->empty()) {
            response->success = false;
            response->message = "No scans accumulated yet";
            return;
        }
        std::string filename = output_directory_ + "/" + position_prefix_ + ".pcd";
        try {
            pcl::io::savePCDFileBinary(filename, *accumulated_cloud_);
            response->success = true;
            response->message = "Saved " + std::to_string(accumulated_cloud_->size()) +
                                " points to " + filename;
            RCLCPP_INFO(this->get_logger(), "[ACCUMULATOR] %s", response->message.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Save failed: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    // ---------------------------------------------------------------
    // Clear
    // ---------------------------------------------------------------
    void clearScansCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        accumulated_cloud_->clear();
        scan_count_          = 0;
        block_scan_count_    = 0;
        completed_block_ready_ = false;
        completed_block_points_ = 0;
        floor_z_initialized_ = false;
        reference_floor_z_   = 0.0f;
        RCLCPP_INFO(this->get_logger(), "[ACCUMULATOR] Cache cleared");
    }

    // ---------------------------------------------------------------
    // Completed block readiness
    // ---------------------------------------------------------------
    void blockReadyCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        response->success = completed_block_ready_;
        if (completed_block_ready_) {
            response->message = "Block #" + std::to_string(current_block_index_ - 1) +
                                " ready on /scanner/merged_cloud (" +
                                std::to_string(completed_block_points_) + " pts)";
        } else {
            response->message = "Waiting for block completion: scan " +
                                std::to_string(block_scan_count_) + "/3, " +
                                std::to_string(accumulated_cloud_->size()) + " pts buffered";
        }
    }

    // ---------------------------------------------------------------
    // Count
    // ---------------------------------------------------------------
    void getScanCountCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        response->success = true;
        response->message = "Scans: " + std::to_string(scan_count_) +
                            ", Points: " + std::to_string(accumulated_cloud_->size());
    }

    // ---------------------------------------------------------------
    // Members
    // ---------------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    merged_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr             save_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr               clear_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr             block_ready_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr             count_service_;
    rclcpp::TimerBase::SharedPtr                                   publish_timer_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Full merged cloud (base_link frame, all points) — also serves as the SLAM map
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud_;
    // Floor Z reference — set from scan #1, reused for all subsequent scans
    // to ensure consistent structure extraction (prevents GICP divergence)
    float reference_floor_z_;
    bool  floor_z_initialized_;
    // TF translation of the previous scan — used to seed the GICP initial guess
    Eigen::Vector3d prev_tf_translation_;
    std::mutex cloud_mutex_;
    int scan_count_;
    int block_scan_count_;
    int current_block_index_ = 1;
    bool completed_block_ready_ = false;
    size_t completed_block_points_ = 0;

    std::string fixed_frame_;
    std::string output_directory_;
    std::string position_prefix_;   // set via: ros2 param set /scan_accumulator position_prefix pos2
    double gicp_max_corr_;
    int    gicp_max_iter_;
    double gicp_downsample_;
    int    gicp_num_threads_;
    std::string overlap_roi_mode_;
    std::string overlap_roi_axis_;
    double overlap_roi_ratio_;
    int    dynamic_roi_bins_;
    double dynamic_roi_score_threshold_;
    int    dynamic_roi_min_bin_points_;
    double floor_band_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanAccumulatorNode>());
    rclcpp::shutdown();
    return 0;
}
