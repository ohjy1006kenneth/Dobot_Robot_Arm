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
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

namespace fs = std::filesystem;

// ============================================================================
// CONFIGURATION CONSTANTS - OPTIMIZED FOR MAXIMUM ACCURACY
// ============================================================================
static const double X_OFFSET_BETWEEN_SCANS_M = 0.65;  // Horizontal robot movement between left/center/right positions (adjust to match actual robot movement)
static const bool USE_ICP_ALIGNMENT = true;           // Enable automatic ICP alignment to refine position based on overlap
static const int ICP_MAX_ITERATIONS = 200;            // Maximum iterations for ICP convergence (high for maximum accuracy)
static const double ICP_MAX_CORRESPONDENCE_DISTANCE = 0.050;  // Max distance for point matching (10mm - very tight for precision alignment)
static const double DOWNSAMPLE_LEAF_SIZE = 0.002;     // Voxel grid size for downsampling (2mm - very fine for maximum detail)
static const int AUTO_PUBLISH_INTERVAL_MS = 1000;     // Publish merged cloud every N milliseconds
static const bool REMOVE_PLANE_TILT = true;           // Remove background plane to flatten tilted scans
// ============================================================================

class ScanAccumulatorNode : public rclcpp::Node
{
public:
    ScanAccumulatorNode() : Node("scan_accumulator"), scan_count_(0), x_offset_per_scan_(0.0)
    {
        // TF2 for coordinate transformations
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscriber to single scans
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scanner/single_scan", 10,
            std::bind(&ScanAccumulatorNode::scanCallback, this, std::placeholders::_1));

        // Publisher for merged cloud (for visualization)
        merged_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/scanner/merged_cloud", 10);

        // Service to save merged cloud to PCD file
        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/scanner/save_merged_cloud",
            std::bind(&ScanAccumulatorNode::saveMergedCloudCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Service to clear accumulated scans
        clear_service_ = this->create_service<std_srvs::srv::Empty>(
            "/scanner/clear_scans",
            std::bind(&ScanAccumulatorNode::clearScansCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Service to get scan count
        count_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/scanner/get_scan_count",
            std::bind(&ScanAccumulatorNode::getScanCountCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Parameters
        this->declare_parameter("target_frame", "laser_frame");
        this->declare_parameter("output_directory", std::string(getenv("HOME")) + "/Dobot_Robot_Arm/scans");
        this->declare_parameter("auto_publish_interval_ms", AUTO_PUBLISH_INTERVAL_MS);
        this->declare_parameter("x_offset_between_scans_m", X_OFFSET_BETWEEN_SCANS_M);
        this->declare_parameter("use_icp_alignment", USE_ICP_ALIGNMENT);
        this->declare_parameter("icp_max_iterations", ICP_MAX_ITERATIONS);
        this->declare_parameter("icp_max_correspondence_distance", ICP_MAX_CORRESPONDENCE_DISTANCE);
        this->declare_parameter("downsample_leaf_size", DOWNSAMPLE_LEAF_SIZE);

        target_frame_ = this->get_parameter("target_frame").as_string();
        output_directory_ = this->get_parameter("output_directory").as_string();
        x_offset_per_scan_ = this->get_parameter("x_offset_between_scans_m").as_double();
        use_icp_alignment_ = this->get_parameter("use_icp_alignment").as_bool();
        icp_max_iterations_ = this->get_parameter("icp_max_iterations").as_int();
        icp_max_correspondence_distance_ = this->get_parameter("icp_max_correspondence_distance").as_double();
        downsample_leaf_size_ = this->get_parameter("downsample_leaf_size").as_double();

        // Create output directory if it doesn't exist
        try {
            fs::create_directories(output_directory_);
            RCLCPP_INFO(this->get_logger(), "Output directory: %s", output_directory_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create output directory: %s", e.what());
            output_directory_ = "/tmp";  // Fallback to /tmp
            RCLCPP_WARN(this->get_logger(), "Using fallback directory: /tmp");
        }

        // Timer to periodically publish merged cloud for visualization
        int publish_interval = this->get_parameter("auto_publish_interval_ms").as_int();
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_interval),
            std::bind(&ScanAccumulatorNode::publishMergedCloud, this));

        // Initialize accumulated cloud with RGB color support
        accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

        RCLCPP_INFO(this->get_logger(), "[ACCUMULATOR] Ready - ICP enabled, X-offset: %.2fm", x_offset_per_scan_);
    }

private:

    // Compute the principal direction (yaw angle) of a point cloud in X-Y plane
    float computePrincipalYaw(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
    {
        if (cloud->size() < 10) {
            return 0.0f;  // Not enough points
        }

        // Compute centroid
        float cx = 0, cy = 0;
        for (const auto& pt : cloud->points) {
            cx += pt.x;
            cy += pt.y;
        }
        cx /= cloud->size();
        cy /= cloud->size();

        // Compute covariance matrix in X-Y plane
        float cxx = 0, cxy = 0, cyy = 0;
        for (const auto& pt : cloud->points) {
            float dx = pt.x - cx;
            float dy = pt.y - cy;
            cxx += dx * dx;
            cxy += dx * dy;
            cyy += dy * dy;
        }
        cxx /= cloud->size();
        cxy /= cloud->size();
        cyy /= cloud->size();

        // Find principal direction using eigenvalue decomposition
        // For 2x2 matrix: eigenvalues = (trace ± sqrt(trace² - 4*det)) / 2
        float trace = cxx + cyy;
        float det = cxx * cyy - cxy * cxy;
        float discriminant = trace * trace - 4 * det;
        
        if (discriminant < 0) {
            return 0.0f;  // Degenerate case
        }

        // Eigenvector for largest eigenvalue gives principal direction
        float lambda_max = (trace + std::sqrt(discriminant)) / 2;
        
        // Eigenvector: [cxy, lambda_max - cxx]
        float vx = cxy;
        float vy = lambda_max - cxx;
        
        if (std::abs(vx) < 1e-6 && std::abs(vy) < 1e-6) {
            return 0.0f;  // No dominant direction
        }

        // Compute angle (yaw) from eigenvector
        return std::atan2(vy, vx);
    }

    void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        int num_points = msg->width * msg->height;

        // Temporary cloud to store incoming scan
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        scan_cloud->points.reserve(num_points);
        scan_cloud->is_dense = true;

        // Find field offsets
        int x_offset = -1, y_offset = -1, z_offset = -1, intensity_offset = -1;
        for (size_t i = 0; i < msg->fields.size(); ++i) {
            if (msg->fields[i].name == "x") x_offset = msg->fields[i].offset;
            if (msg->fields[i].name == "y") y_offset = msg->fields[i].offset;
            if (msg->fields[i].name == "z") z_offset = msg->fields[i].offset;
            if (msg->fields[i].name == "intensity") intensity_offset = msg->fields[i].offset;
        }

        // Parse point cloud data (no coloring yet - will color AFTER plane fitting)
        int valid_count = 0;
        for (int i = 0; i < num_points; ++i) {
            const uint8_t* data_ptr = &msg->data[i * msg->point_step];
            
            float x = *reinterpret_cast<const float*>(data_ptr + x_offset);
            float y = *reinterpret_cast<const float*>(data_ptr + y_offset);
            float z = *reinterpret_cast<const float*>(data_ptr + z_offset);

            // Only add valid points
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && 
                z != 0.0f && z > -0.999f) {
                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.r = 128;  // Temporary gray color
                point.g = 128;
                point.b = 128;
                
                scan_cloud->points.push_back(point);
                valid_count++;
            }
        }

        scan_cloud->width = valid_count;
        scan_cloud->height = 1;

        // DON'T do plane fitting on individual scans - we'll do it on the merged cloud instead
        // This ensures all scans share the same reference plane

        // Apply initial X offset for each scan (left, center, right positioning)
        double initial_x_offset = (scan_count_) * x_offset_per_scan_;
        
        if (scan_count_ == 0 || !use_icp_alignment_ || accumulated_cloud_->empty()) {
            // First scan or ICP disabled: just apply the configured offset
            for (auto& point : scan_cloud->points) {
                point.x += initial_x_offset;
            }
            *accumulated_cloud_ += *scan_cloud;
            
            RCLCPP_INFO(this->get_logger(), 
                        "[ACCUMULATOR] Scan #%d: %d pts → Total: %lu pts", 
                        scan_count_ + 1, valid_count, accumulated_cloud_->size());
        } else {
            // Hybrid approach: Use configured offset as initial guess, then refine with ICP
            RCLCPP_INFO(this->get_logger(), 
                        "[ACCUMULATOR] Scan #%d: Aligning %d pts (ICP)...", 
                        scan_count_ + 1, valid_count);
            
            // Apply initial offset
            Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();
            initial_transform(0, 3) = initial_x_offset;  // X translation
            
            // AUTO ROTATION ALIGNMENT: Detect and correct yaw differences
            float scan_yaw = computePrincipalYaw(scan_cloud);
            float accumulated_yaw = computePrincipalYaw(accumulated_cloud_);
            float yaw_correction = accumulated_yaw - scan_yaw;
            
            // Handle 180° ambiguity: PCA can't distinguish 0° from 180°
            while (yaw_correction > M_PI) yaw_correction -= 2 * M_PI;
            while (yaw_correction < -M_PI) yaw_correction += 2 * M_PI;
            
            // If correction is close to ±180°, flip to opposite sign
            if (yaw_correction > M_PI / 2) yaw_correction -= M_PI;
            if (yaw_correction < -M_PI / 2) yaw_correction += M_PI;
            
            // Only apply if rotation is reasonable for horizontal movement (<15°)
            if (std::abs(yaw_correction) < 15.0 * M_PI / 180.0) {
                // Apply rotation around Z-axis
                float cos_yaw = std::cos(yaw_correction);
                float sin_yaw = std::sin(yaw_correction);
                
                Eigen::Matrix4f rotation_transform = Eigen::Matrix4f::Identity();
                rotation_transform(0, 0) = cos_yaw;
                rotation_transform(0, 1) = -sin_yaw;
                rotation_transform(1, 0) = sin_yaw;
                rotation_transform(1, 1) = cos_yaw;
                
                // Combine rotation and translation
                initial_transform = initial_transform * rotation_transform;
                
                RCLCPP_INFO(this->get_logger(), 
                            "[ACCUMULATOR] Pre-aligned yaw rotation: %.2f°",
                            yaw_correction * 180.0 / M_PI);
            }
            
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_scan(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*scan_cloud, *transformed_scan, initial_transform);
            
            // Downsample both clouds for faster ICP
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
            
            pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
            voxel_filter.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
            
            voxel_filter.setInputCloud(transformed_scan);
            voxel_filter.filter(*source_downsampled);
            
            voxel_filter.setInputCloud(accumulated_cloud_);
            voxel_filter.filter(*target_downsampled);
            
            // Run ICP with maximum accuracy settings
            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
            icp.setMaximumIterations(icp_max_iterations_);
            icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
            icp.setTransformationEpsilon(1e-10);  // Very strict convergence threshold for accuracy
            icp.setEuclideanFitnessEpsilon(1e-6); // Very strict fitness threshold for accuracy
            icp.setRANSACOutlierRejectionThreshold(0.005);  // 5mm outlier rejection for cleaner alignment
            
            icp.setInputSource(source_downsampled);
            icp.setInputTarget(target_downsampled);
            
            pcl::PointCloud<pcl::PointXYZRGB> aligned_downsampled;
            icp.align(aligned_downsampled);
            
            if (icp.hasConverged()) {
                // Apply the refined transformation to the full-resolution scan
                Eigen::Matrix4f final_transform = icp.getFinalTransformation();
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_scan(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::transformPointCloud(*transformed_scan, *aligned_scan, final_transform);
                
                // Extract the actual offsets after ICP refinement
                float refined_x_offset = initial_x_offset + final_transform(0, 3);
                float y_offset_detected = final_transform(1, 3);
                
                *accumulated_cloud_ += *aligned_scan;
                
                // Log Y-offset if significant (timing difference between scan starts)
                if (std::abs(y_offset_detected) > 0.005) {
                    RCLCPP_INFO(this->get_logger(), 
                                "[ACCUMULATOR] ✓ ICP aligned: X=%.3fm (Δ%.3fm), Y=%.3fm (timing), Score: %.6f → Total: %lu pts", 
                                refined_x_offset, final_transform(0, 3), y_offset_detected,
                                icp.getFitnessScore(), accumulated_cloud_->size());
                } else {
                    RCLCPP_INFO(this->get_logger(), 
                                "[ACCUMULATOR] ✓ ICP aligned: X=%.3fm (Δ%.3fm), Score: %.6f → Total: %lu pts", 
                                refined_x_offset, final_transform(0, 3),
                                icp.getFitnessScore(), accumulated_cloud_->size());
                }
            } else {
                // ICP failed, fall back to configured offset
                RCLCPP_WARN(this->get_logger(), "[ACCUMULATOR] ⚠ ICP failed - using initial offset");
                *accumulated_cloud_ += *transformed_scan;
            }
        }
        
        scan_count_++;
        
        // Only recolor when saving, not during accumulation (too slow with millions of points)
        // Colors will be updated when user saves the final merged cloud
    }

    void publishMergedCloud()
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        if (accumulated_cloud_->empty()) {
            return;  // Don't publish empty clouds
        }

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*accumulated_cloud_, output_msg);
        output_msg.header.frame_id = target_frame_;
        output_msg.header.stamp = this->get_clock()->now();

        merged_publisher_->publish(output_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published merged cloud with %lu points", 
                     accumulated_cloud_->size());
    }

    void saveMergedCloudCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        if (accumulated_cloud_->empty()) {
            response->success = false;
            response->message = "No scans accumulated yet";
            RCLCPP_WARN(this->get_logger(), "No scans to save");
            return;
        }

        // Recolor based on actual Z-range after plane fitting for better crack visualization
        removePlaneFromMergedCloud();  // First remove global plane tilt
        recolorMergedCloud();          // Then color by Z-height

        // Generate filename with timestamp
        auto now = this->get_clock()->now();
        std::string filename = output_directory_ + "/crack_scan_" + 
                              std::to_string(now.seconds()) + ".pcd";

        try {
            pcl::io::savePCDFileBinary(filename, *accumulated_cloud_);
            
            response->success = true;
            response->message = "Saved " + std::to_string(accumulated_cloud_->size()) + 
                               " points to " + filename;

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Failed to save: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Save failed: %s", e.what());
        }
    }

    void removePlaneFromMergedCloud()
    {
        if (!accumulated_cloud_ || accumulated_cloud_->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot fit plane: merged cloud is empty");
            return;
        }

        const bool REMOVE_PLANE_TILT = true;
        if (!REMOVE_PLANE_TILT) return;

        int valid_count = 0;
        for (const auto& pt : accumulated_cloud_->points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                valid_count++;
            }
        }

        if (valid_count < 100) {
            RCLCPP_WARN(this->get_logger(), "Not enough valid points (%d) to fit plane", valid_count);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Fitting plane to entire merged cloud (%d valid points)...", valid_count);

        // Fit plane using least squares: z = ax + by + c
        Eigen::MatrixXd A(valid_count, 3);
        Eigen::VectorXd b(valid_count);
        
        int row = 0;
        for (const auto& pt : accumulated_cloud_->points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                A(row, 0) = pt.x;
                A(row, 1) = pt.y;
                A(row, 2) = 1.0;
                b(row) = pt.z;
                row++;
            }
        }

        // Solve using normal equations: (A^T A)x = A^T b
        Eigen::Vector3d plane_params = (A.transpose() * A).ldlt().solve(A.transpose() * b);
        double a = plane_params(0);
        double b_coef = plane_params(1);
        double c = plane_params(2);

        RCLCPP_INFO(this->get_logger(), "Fitted plane: z = %.4fx + %.4fy + %.4f", a, b_coef, c);

        // Subtract plane from all points (keep x,y but adjust z) - MULTI-THREADED
        size_t num_points = accumulated_cloud_->points.size();
        #pragma omp parallel for
        for (size_t i = 0; i < num_points; ++i) {
            auto& pt = accumulated_cloud_->points[i];
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                float plane_z = a * pt.x + b_coef * pt.y + c;
                pt.z -= plane_z;  // Remove the plane, leaving only surface variations
            }
        }

        RCLCPP_INFO(this->get_logger(), "Removed global plane tilt from merged cloud (multi-threaded)");
    }

    void recolorMergedCloud()
    {
        if (accumulated_cloud_->empty()) {
            return;
        }

        // Find actual Z range in merged cloud
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();
        
        for (const auto& pt : accumulated_cloud_->points) {
            if (pt.z < min_z) min_z = pt.z;
            if (pt.z > max_z) max_z = pt.z;
        }
        
        float z_range = max_z - min_z;
        
        RCLCPP_INFO(this->get_logger(), 
                    "[ACCUMULATOR] Coloring: Z=[%.6f, %.6f]m (range=%.6fm = %.3fmm)", 
                    min_z, max_z, z_range, z_range * 1000.0);
        RCLCPP_INFO(this->get_logger(), 
                    "[ACCUMULATOR] Blue=LOWEST (%.6fm), Red=HIGHEST (%.6fm)", 
                    min_z, max_z);

        // Recolor all points: Blue (lowest Z) → Red (highest Z) - MULTI-THREADED
        size_t num_points = accumulated_cloud_->points.size();
        #pragma omp parallel for
        for (size_t i = 0; i < num_points; ++i) {
            auto& pt = accumulated_cloud_->points[i];
            
            // Normalize Z to [0, 1] where 0=lowest, 1=highest
            float z_normalized = (z_range > 0.0001f) ? 
                                (pt.z - min_z) / z_range : 0.5f;
            
            // Rainbow colormap: Blue (0) → Cyan → Green → Yellow → Red (1)
            float r, g, b;
            if (z_normalized < 0.25f) {
                // Blue to Cyan
                r = 0;
                g = z_normalized * 4.0f;
                b = 1.0f;
            } else if (z_normalized < 0.5f) {
                // Cyan to Green
                r = 0;
                g = 1.0f;
                b = 1.0f - (z_normalized - 0.25f) * 4.0f;
            } else if (z_normalized < 0.75f) {
                // Green to Yellow
                r = (z_normalized - 0.5f) * 4.0f;
                g = 1.0f;
                b = 0;
            } else {
                // Yellow to Red
                r = 1.0f;
                g = 1.0f - (z_normalized - 0.75f) * 4.0f;
                b = 0;
            }
            
            pt.r = static_cast<uint8_t>(r * 255);
            pt.g = static_cast<uint8_t>(g * 255);
            pt.b = static_cast<uint8_t>(b * 255);
        }
        
        RCLCPP_INFO(this->get_logger(), "[ACCUMULATOR] Recolored %zu points (multi-threaded)", num_points);
    }

    void clearScansCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        accumulated_cloud_->clear();
        scan_count_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "[ACCUMULATOR] Cache cleared");
    }

    void getScanCountCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        response->success = true;
        response->message = "Scans: " + std::to_string(scan_count_) + 
                           ", Points: " + std::to_string(accumulated_cloud_->size());
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr count_service_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;
    std::mutex cloud_mutex_;
    int scan_count_;
    double x_offset_per_scan_;
    bool use_icp_alignment_;
    int icp_max_iterations_;
    double icp_max_correspondence_distance_;
    double downsample_leaf_size_;

    std::string target_frame_;
    std::string output_directory_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanAccumulatorNode>());
    rclcpp::shutdown();
    return 0;
}