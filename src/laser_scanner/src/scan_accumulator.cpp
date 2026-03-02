#include <memory>
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
static const double GICP_MAX_CORRESPONDENCE_DISTANCE = 0.10;  // 100 mm — wide enough for 55% overlapping strips
static const int    GICP_MAX_ITERATIONS              = 200;
static const double GICP_DOWNSAMPLING_RESOLUTION     = 0.005; // 5 mm voxel (more points survive)
static const int    GICP_NUM_THREADS                 = 4;
static const int    GICP_NUM_NEIGHBORS               = 20;

// Floor filtering for GICP input.
// Points whose Z (in base_link) is within FLOOR_BAND_M of the detected
// floor level are excluded from registration.  Full-resolution merged
// cloud always keeps all points.
// NOTE: floor_z is detected from scan #1 and reused for all subsequent scans
//       to ensure consistent structure extraction across all strips.
static const double FLOOR_BAND_M            = 0.020;  // +-20 mm around floor = removed
static const double FLOOR_DETECT_PERCENTILE = 0.02;   // bottom 2% of Z = floor level
// ============================================================================

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

class ScanAccumulatorNode : public rclcpp::Node
{
public:
    ScanAccumulatorNode() : Node("scan_accumulator"), scan_count_(0)
    {
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scanner/single_scan", 10,
            std::bind(&ScanAccumulatorNode::scanCallback, this, std::placeholders::_1));

        merged_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/scanner/merged_cloud", 10);

        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/scanner/save_merged_cloud",
            std::bind(&ScanAccumulatorNode::saveMergedCloudCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        clear_service_ = this->create_service<std_srvs::srv::Empty>(
            "/scanner/clear_scans",
            std::bind(&ScanAccumulatorNode::clearScansCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        count_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/scanner/get_scan_count",
            std::bind(&ScanAccumulatorNode::getScanCountCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Parameters
        this->declare_parameter("fixed_frame", "base_link");
        this->declare_parameter("output_directory",
                                std::string(getenv("HOME")) + "/Dobot_Robot_Arm/scans");
        this->declare_parameter("auto_publish_interval_ms", AUTO_PUBLISH_INTERVAL_MS);
        this->declare_parameter("gicp_max_correspondence_distance", GICP_MAX_CORRESPONDENCE_DISTANCE);
        this->declare_parameter("gicp_max_iterations",              GICP_MAX_ITERATIONS);
        this->declare_parameter("gicp_downsampling_resolution",     GICP_DOWNSAMPLING_RESOLUTION);
        this->declare_parameter("gicp_num_threads",                 GICP_NUM_THREADS);
        this->declare_parameter("floor_band_m",                     FLOOR_BAND_M);

        fixed_frame_      = this->get_parameter("fixed_frame").as_string();
        output_directory_ = this->get_parameter("output_directory").as_string();
        gicp_max_corr_    = this->get_parameter("gicp_max_correspondence_distance").as_double();
        gicp_max_iter_    = this->get_parameter("gicp_max_iterations").as_int();
        gicp_downsample_  = this->get_parameter("gicp_downsampling_resolution").as_double();
        gicp_num_threads_ = this->get_parameter("gicp_num_threads").as_int();
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
            "[ACCUMULATOR] Ready — structure-GICP + TF, fixed_frame=%s, "
            "corr=%.3fm, floor_band=+-%.1fmm",
            fixed_frame_.c_str(), gicp_max_corr_, floor_band_ * 1000.0);
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
            pt.intensity = (i_off >= 0) ? static_cast<float>(ptr[i_off]) : 0.0f;
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

        // 1. Parse
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
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_in_fixed(
            new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*scan_pcl, *scan_in_fixed, tf_matrix);

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

        // 5. First scan -- store directly, no registration needed
        if (scan_count_ == 0 || accumulated_cloud_->empty()) {
            *accumulated_cloud_ += *scan_in_fixed;
            prev_tf_translation_ = tf_t;
            scan_count_++;
            RCLCPP_INFO(this->get_logger(),
                "[ACCUMULATOR] Scan #1 stored (TF %s) -> Total: %zu pts",
                have_tf ? "OK" : "IDENTITY-FALLBACK",
                accumulated_cloud_->size());
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
            prev_tf_translation_ = tf_t;
            scan_count_++;
            RCLCPP_WARN(this->get_logger(),
                "[ACCUMULATOR] Scan #%d: too few structure pts -- TF-only -> Total: %zu pts",
                scan_count_, accumulated_cloud_->size());
            return;
        }

        auto sg_target = toSmallGicp3D(map_structure);
        auto sg_source = toSmallGicp3D(scan_structure);

        auto [target_ds, target_tree] = small_gicp::preprocess_points(
            *sg_target, gicp_downsample_, GICP_NUM_NEIGHBORS, gicp_num_threads_);
        auto [source_ds, source_tree] = small_gicp::preprocess_points(
            *sg_source, gicp_downsample_, GICP_NUM_NEIGHBORS, gicp_num_threads_);

        RCLCPP_INFO(this->get_logger(),
            "[ACCUMULATOR] GICP #%d: map_ds=%zu  scan_ds=%zu  corr=%.3fm",
            scan_count_ + 1, target_ds->size(), source_ds->size(), gicp_max_corr_);

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
        constexpr double MAX_GICP_TRANSLATION_M = 0.15;
        constexpr double MAX_GICP_ROTATION_DEG  = 10.0;

        if (result.converged && result.iterations > 0 &&
            dt_m < MAX_GICP_TRANSLATION_M && dr_deg < MAX_GICP_ROTATION_DEG)
        {
            Eigen::Matrix4f correction = result.T_target_source.matrix().cast<float>();
            pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*scan_in_fixed, *aligned, correction);
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

        prev_tf_translation_ = tf_t;
        scan_count_++;
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
        std::string filename = output_directory_ + "/laser_scan_" +
                               std::to_string(this->get_clock()->now().seconds()) + ".pcd";
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
        floor_z_initialized_ = false;
        reference_floor_z_   = 0.0f;
        RCLCPP_INFO(this->get_logger(), "[ACCUMULATOR] Cache cleared");
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

    std::string fixed_frame_;
    std::string output_directory_;
    double gicp_max_corr_;
    int    gicp_max_iter_;
    double gicp_downsample_;
    int    gicp_num_threads_;
    double floor_band_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanAccumulatorNode>());
    rclcpp::shutdown();
    return 0;
}
