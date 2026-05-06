// map_merger.cpp
//
// Accumulates 3-strip blocks (from scan_accumulator) into a growing world map
// using manually-configured rover positions (X, Y in metres, yaw in degrees).
//
// Each call to /map_merger/add_block consumes the next position in the list.
// Block #1 uses position[0], Block #2 uses position[1], etc.
//
// Positions are set as ROS2 parameters:
//   positions_x: [0.0, 1.0, 2.0]   # X offsets in metres
//   positions_y: [0.0, -1.0, 0.0]  # Y offsets in metres
//   positions_yaw: [0.0, 0.0, 0.0] # Yaw offsets in degrees (rover heading change)
//
// Services:
//   /map_merger/add_block  -- place current block using next configured position
//   /map_merger/save       -- save merged_map.pcd
//   /map_merger/reset      -- clear world map and reset position index
//
// Topics subscribed:
//   /scanner/merged_cloud  (sensor_msgs/PointCloud2, frame=base_link)
//
// Topics published:
//   /map_merger/world_map  (sensor_msgs/PointCloud2, frame=odom)

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <filesystem>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// small_gicp headers (native API)
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/registration/registration_helper.hpp>

namespace fs = std::filesystem;

// ============================================================================
// GICP settings for inter-block merging
// ============================================================================
static const double MM_GICP_MAX_CORR        = 0.15;   // 150 mm — blocks share surface geometry
static const int    MM_GICP_MAX_ITER        = 200;
static const double MM_GICP_DOWNSAMPLE      = 0.005;  // 5 mm voxel
static const int    MM_GICP_NUM_THREADS     = 4;
static const int    MM_GICP_NUM_NEIGHBORS   = 20;
static const double MM_GICP_MAX_TRANS_M     = 0.20;   // reject if GICP moves >20cm
static const double MM_GICP_MAX_ROT_DEG     = 5.0;    // reject if GICP rotates >5deg
// ============================================================================

// ---------------------------------------------------------------------------
// PCL -> small_gicp::PointCloud
// ---------------------------------------------------------------------------
[[maybe_unused]]
static std::shared_ptr<small_gicp::PointCloud> toSmallGicp3D(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in)
{
    auto sg = std::make_shared<small_gicp::PointCloud>();
    sg->resize(in->size());
    for (size_t i = 0; i < in->size(); ++i)
        sg->point(i) << in->points[i].x, in->points[i].y, in->points[i].z, 1.0;
    return sg;
}

class MapMergerNode : public rclcpp::Node
{
public:
    MapMergerNode() : Node("map_merger"), block_count_(0)
    {
        // Output directory
        this->declare_parameter("output_directory",
            std::string(getenv("HOME")) + "/Dobot_Robot_Arm/scans");
        output_directory_ = this->get_parameter("output_directory").as_string();
        fs::create_directories(output_directory_);

        // Hardcoded rover positions: X (forward), Y (left), yaw (degrees)
        // Position #1 = origin (0,0,0) — where the rover starts
        // Position #2 = 1m right, 1m backward → X=-1, Y=-1 (rover frame: X=forward, Y=left)
        // Position #3 = 1m right, 1m forward  → X=+1, Y=-1 relative to start
        // Adjust signs to match your rover's coordinate frame convention.
        this->declare_parameter("positions_x",   std::vector<double>{0.0, -1.0,  0.0});
        this->declare_parameter("positions_y",   std::vector<double>{0.0, -1.0, -2.0});
        this->declare_parameter("positions_yaw", std::vector<double>{0.0,  0.0,  0.0});

        positions_x_   = this->get_parameter("positions_x").as_double_array();
        positions_y_   = this->get_parameter("positions_y").as_double_array();
        positions_yaw_ = this->get_parameter("positions_yaw").as_double_array();

        // Validate
        if (positions_x_.size() != positions_y_.size() ||
            positions_x_.size() != positions_yaw_.size()) {
            RCLCPP_ERROR(this->get_logger(),
                "[MAP_MERGER] positions_x/y/yaw must have the same length! Got %zu/%zu/%zu",
                positions_x_.size(), positions_y_.size(), positions_yaw_.size());
        }

        block_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scanner/merged_cloud",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            std::bind(&MapMergerNode::blockCloudCallback, this, std::placeholders::_1));

        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/map_merger/world_map", 1);

        add_block_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/map_merger/add_block",
            std::bind(&MapMergerNode::addBlockCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        save_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/map_merger/save",
            std::bind(&MapMergerNode::saveCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        reset_srv_ = this->create_service<std_srvs::srv::Empty>(
            "/map_merger/reset",
            std::bind(&MapMergerNode::resetCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        clear_client_ = this->create_client<std_srvs::srv::Empty>("/scanner/clear_scans");
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "scan_accumulator");
        world_map_    = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        RCLCPP_INFO(this->get_logger(),
            "[MAP_MERGER] Ready -- %zu manual positions configured",
            positions_x_.size());
        for (size_t i = 0; i < positions_x_.size(); i++) {
            RCLCPP_INFO(this->get_logger(),
                "  Position #%zu: X=%.3f m  Y=%.3f m  Yaw=%.1f deg",
                i + 1, positions_x_[i], positions_y_[i], positions_yaw_[i]);
        }
        RCLCPP_INFO(this->get_logger(),
            "  /map_merger/add_block -> place next block\n"
            "  /map_merger/save      -> save merged_map.pcd\n"
            "  /map_merger/reset     -> clear map and reset position index");
    }

private:
    void blockCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_block_msg_ = msg;
    }

    void addBlockCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!latest_block_msg_) {
            response->success = false;
            response->message = "No block on /scanner/merged_cloud yet";
            return;
        }

        if (block_count_ >= static_cast<int>(positions_x_.size())) {
            response->success = false;
            response->message = "All " + std::to_string(positions_x_.size()) +
                                " configured positions already used. "
                                "Call /map_merger/reset or add more positions.";
            RCLCPP_ERROR(this->get_logger(), "[MAP_MERGER] %s", response->message.c_str());
            return;
        }

        // Get the configured position for this block
        double px  = positions_x_[block_count_];
        double py  = positions_y_[block_count_];
        double yaw = positions_yaw_[block_count_] * M_PI / 180.0;  // deg -> rad

        // Build the transform: world_T_rover (position of rover in world/odom frame)
        Eigen::Isometry3d world_T_rover = Eigen::Isometry3d::Identity();
        world_T_rover.translation() << px, py, 0.0;
        world_T_rover.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                                    .toRotationMatrix();

        block_count_++;

        auto block_base = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*latest_block_msg_, *block_base);

        if (block_base->empty()) {
            response->success = false;
            response->message = "Block cloud is empty";
            return;
        }

        // Save this block's cloud as posN.pcd (in base_link frame, before world transform)
        {
            std::string pos_path = output_directory_ + "/pos" + std::to_string(block_count_) + ".pcd";
            try {
                pcl::io::savePCDFileBinary(pos_path, *block_base);
                RCLCPP_INFO(this->get_logger(),
                    "[MAP_MERGER] Saved block #%d (base_link frame) -> %s (%zu pts)",
                    block_count_, pos_path.c_str(), block_base->size());
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "[MAP_MERGER] Failed to save %s: %s",
                    pos_path.c_str(), e.what());
            }
        }

        // Transform block from base_link into world frame using configured position
        auto block_world = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::transformPointCloud(*block_base, *block_world,
            world_T_rover.matrix().cast<float>());

        // Log bounding box of the block in world frame so you can verify positions
        {
            float bx0=1e9,bx1=-1e9, by0=1e9,by1=-1e9, bz0=1e9,bz1=-1e9;
            for (const auto& p : block_world->points) {
                bx0=std::min(bx0,p.x); bx1=std::max(bx1,p.x);
                by0=std::min(by0,p.y); by1=std::max(by1,p.y);
                bz0=std::min(bz0,p.z); bz1=std::max(bz1,p.z);
            }
            RCLCPP_INFO(this->get_logger(),
                "[MAP_MERGER] Block #%d world bbox: X[%.3f..%.3f] Y[%.3f..%.3f] Z[%.3f..%.3f]  pts=%zu",
                block_count_, bx0,bx1, by0,by1, bz0,bz1, block_world->size());

            // If world map already exists, also log overlap check
            if (!world_map_->empty()) {
                float mx0=1e9,mx1=-1e9, my0=1e9,my1=-1e9;
                for (const auto& p : world_map_->points) {
                    mx0=std::min(mx0,p.x); mx1=std::max(mx1,p.x);
                    my0=std::min(my0,p.y); my1=std::max(my1,p.y);
                }
                float ox0=std::max(bx0,mx0), ox1=std::min(bx1,mx1);
                float oy0=std::max(by0,my0), oy1=std::min(by1,my1);
                bool overlap = (ox1>ox0) && (oy1>oy0);
                RCLCPP_INFO(this->get_logger(),
                    "[MAP_MERGER] World map bbox: X[%.3f..%.3f] Y[%.3f..%.3f]  "
                    "XY overlap: %s  overlap region: X[%.3f..%.3f] Y[%.3f..%.3f]",
                    mx0,mx1, my0,my1,
                    overlap?"YES (GICP will run)":"NO (GICP skipped, manual only)",
                    ox0,ox1, oy0,oy1);
            }
        }

        // GICP refinement on top of the manual position (only for block #2 onward,
        // since we need an existing world map as the target).
        // Only attempt GICP if bounding boxes overlap — otherwise there are no correspondences.
        bool gicp_used = false;
        if (!world_map_->empty() && block_world->size() >= 200 && world_map_->size() >= 200) {
            // Quick bounding-box overlap test before running expensive GICP
            float bx0=1e9,bx1=-1e9, by0=1e9,by1=-1e9;
            float mx0=1e9,mx1=-1e9, my0=1e9,my1=-1e9;
            for (const auto& p : block_world->points) {
                bx0=std::min(bx0,p.x); bx1=std::max(bx1,p.x);
                by0=std::min(by0,p.y); by1=std::max(by1,p.y);
            }
            for (const auto& p : world_map_->points) {
                mx0=std::min(mx0,p.x); mx1=std::max(mx1,p.x);
                my0=std::min(my0,p.y); my1=std::max(my1,p.y);
            }
            // Add the GICP max correspondence distance as margin to the overlap test
            float margin = static_cast<float>(MM_GICP_MAX_CORR);
            bool has_overlap = (bx1+margin > mx0) && (bx0-margin < mx1) &&
                               (by1+margin > my0) && (by0-margin < my1);
            if (!has_overlap) {
                RCLCPP_WARN(this->get_logger(),
                    "[MAP_MERGER] Block #%d: no XY overlap with existing map → skipping GICP, using manual position only",
                    block_count_);
            } else {
            auto sg_target = toSmallGicp3D(world_map_);
            auto sg_source = toSmallGicp3D(block_world);

            auto [target_ds, target_tree] = small_gicp::preprocess_points(
                *sg_target, MM_GICP_DOWNSAMPLE, MM_GICP_NUM_NEIGHBORS, MM_GICP_NUM_THREADS);
            auto [source_ds, source_tree] = small_gicp::preprocess_points(
                *sg_source, MM_GICP_DOWNSAMPLE, MM_GICP_NUM_NEIGHBORS, MM_GICP_NUM_THREADS);

            small_gicp::RegistrationSetting setting;
            setting.type = small_gicp::RegistrationSetting::GICP;
            setting.max_correspondence_distance = MM_GICP_MAX_CORR;
            setting.max_iterations = MM_GICP_MAX_ITER;
            setting.num_threads = MM_GICP_NUM_THREADS;

            // Init = Identity: block_world is already placed at the manual position,
            // so GICP starts from there and only corrects small residuals.
            Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
            auto result = small_gicp::align(*target_ds, *source_ds, *target_tree, init_T, setting);

            Eigen::Vector3d dt = result.T_target_source.translation();
            Eigen::AngleAxisd aa(result.T_target_source.rotation());
            double dt_m   = dt.norm();
            double dr_deg = aa.angle() * 180.0 / M_PI;

            if (result.converged && result.iterations > 0 &&
                dt_m < MM_GICP_MAX_TRANS_M && dr_deg < MM_GICP_MAX_ROT_DEG)
            {
                Eigen::Matrix4f correction = result.T_target_source.matrix().cast<float>();
                auto block_refined = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
                pcl::transformPointCloud(*block_world, *block_refined, correction);
                *world_map_ += *block_refined;
                RCLCPP_INFO(this->get_logger(),
                    "[MAP_MERGER] Block #%d GICP OK  iters=%zu  dt=%.4fm  dr=%.3fdeg -> %zu pts total",
                    block_count_, result.iterations, dt_m, dr_deg, world_map_->size());
                gicp_used = true;
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "[MAP_MERGER] Block #%d GICP %s (iters=%zu dt=%.4fm dr=%.2fdeg) -- using manual position",
                    block_count_,
                    (!result.converged) ? "FAIL" : (result.iterations == 0) ? "NO-OVERLAP" : "REJECTED",
                    result.iterations, dt_m, dr_deg);
            }
            } // end if (has_overlap)
        } // end if (!world_map_->empty()...)

        if (!gicp_used) {
            *world_map_ += *block_world;
        }

        RCLCPP_INFO(this->get_logger(),
            "[MAP_MERGER] Block #%d placed at (X=%.3fm, Y=%.3fm, Yaw=%.1fdeg) %s-> %zu pts total",
            block_count_, px, py, positions_yaw_[block_count_ - 1],
            gicp_used ? "[GICP-refined] " : "[manual-only] ",
            world_map_->size());

        if (clear_client_->wait_for_service(std::chrono::seconds(1))) {
            clear_client_->async_send_request(
                std::make_shared<std_srvs::srv::Empty::Request>());
        } else {
            RCLCPP_WARN(this->get_logger(), "[MAP_MERGER] /scanner/clear_scans unavailable");
        }

        // Update scan_accumulator's position_prefix so next block's individual scans
        // are named posN_scan1.pcd, posN_scan2.pcd, posN_scan3.pcd
        {
            std::string next_prefix = "pos" + std::to_string(block_count_ + 1);
            if (param_client_->service_is_ready()) {
                param_client_->set_parameters(
                    {rclcpp::Parameter("position_prefix", next_prefix)},
                    [this, next_prefix](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> fut) {
                        (void)fut;
                        RCLCPP_INFO(this->get_logger(),
                            "[MAP_MERGER] scan_accumulator position_prefix -> %s", next_prefix.c_str());
                    });
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "[MAP_MERGER] scan_accumulator param service not ready -- "
                    "next scans will still use previous prefix. "
                    "Use: ros2 param set /scan_accumulator position_prefix %s",
                    next_prefix.c_str());
            }
        }
        latest_block_msg_ = nullptr;

        publishWorldMap();
        response->success = true;
        response->message = "Block #" + std::to_string(block_count_) +
                            " placed at (" + std::to_string(px) + ", " +
                            std::to_string(py) + ") m, world map: " +
                            std::to_string(world_map_->size()) + " pts";
    }

    void saveCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (world_map_->empty()) {
            response->success = false;
            response->message = "World map empty";
            return;
        }
        std::string path = output_directory_ + "/merged_map.pcd";
        try {
            pcl::io::savePCDFileBinary(path, *world_map_);
            response->success = true;
            response->message = "Saved " + std::to_string(world_map_->size()) +
                                " pts -> " + path;
            RCLCPP_INFO(this->get_logger(), "[MAP_MERGER] %s", response->message.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Save failed: ") + e.what();
        }
    }

    void resetCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        world_map_->clear();
        block_count_ = 0;
        latest_block_msg_ = nullptr;
        RCLCPP_INFO(this->get_logger(),
            "[MAP_MERGER] World map reset -- position index back to 0");

        // Reset scan_accumulator prefix back to pos1
        if (param_client_->service_is_ready()) {
            param_client_->set_parameters(
                {rclcpp::Parameter("position_prefix", std::string("pos1"))},
                [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> fut) {
                    (void)fut;
                    RCLCPP_INFO(this->get_logger(),
                        "[MAP_MERGER] scan_accumulator position_prefix reset -> pos1");
                });
        }
    }

    void publishWorldMap()
    {
        if (world_map_->empty()) return;
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(*world_map_, out);
        out.header.frame_id = "odom";
        out.header.stamp    = this->get_clock()->now();
        map_pub_->publish(out);
    }

    std::string          output_directory_;
    int                  block_count_;
    std::vector<double>  positions_x_;
    std::vector<double>  positions_y_;
    std::vector<double>  positions_yaw_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr block_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     map_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              add_block_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              save_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                reset_srv_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr                 clear_client_;
    rclcpp::AsyncParametersClient::SharedPtr                        param_client_;

    std::mutex mutex_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_block_msg_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr     world_map_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMergerNode>());
    rclcpp::shutdown();
    return 0;
}
