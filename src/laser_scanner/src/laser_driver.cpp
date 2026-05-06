#include <memory>
#include <limits>
#include <cmath>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "KSJApi3D.h"

// Scanner configuration constants
constexpr int TARGET_PROFILES = 1500;       // MAXIMUM Y-resolution (10x finer than original 1000)
constexpr int MAX_SCAN_RATE_HZ = 500;        // Scanner hardware maximum frequency (realistic limit)

// Optical parameters
constexpr double EXPOSURE_TIME_MS = 1.8;     // Balanced exposure for 1m distance
constexpr int GAIN = 18;                     // Moderate gain - reduces noise
constexpr int BRIGHTNESS_THRESHOLD = 10;     // Higher threshold filters noise (increase to 12-15 if still noisy)
constexpr int BRIGHTNESS_LOW_THRESHOLD = 255; // Low brightness cutoff (255 = disabled)
constexpr int LASER_LINE_WIDTH = 35;         // Tighter tolerance reduces noise

// Robot movement parameters
constexpr double Y_TRAVEL_DISTANCE_MM = 230.0;  // Physical distance scanner moves (mm)
constexpr double ROBOT_SPEED_M_S = 0.04;          // Robot movement speed (m/s)
constexpr double Y_RESOLUTION_MM = 0.15;          // Y-axis sampling resolution sent to hardware (mm)

// Working distance guidelines:
// Close range (<500mm): EXPOSURE=1.0, GAIN=10, THRESHOLD=8, WIDTH=25
// Medium range (500-800mm): EXPOSURE=1.5, GAIN=15, THRESHOLD=6, WIDTH=35
// **Optimal (800-1200mm): EXPOSURE=1.8, GAIN=18, THRESHOLD=10-12, WIDTH=35** ← YOUR CURRENT SETUP
// Far range (>1200mm): EXPOSURE=2.5-3.0, GAIN=20-25, THRESHOLD=4-6, WIDTH=50

float g_fXStart, g_fXRes;
int g_nUniformWidth;

class KsjDriverNode : public rclcpp::Node
{
public:
    KsjDriverNode() : Node("ksj_driver"), scanning_(false), scan_received_(false)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/scanner/single_scan", 10);

        if (!initializeScanner()) {
            RCLCPP_FATAL(this->get_logger(),
                         "[SCANNER] Initialization failed. /scanner/trigger_scan will not be advertised.");
            throw std::runtime_error("KSJ scanner initialization failed");
        }

        // Service to trigger a single scan
        scan_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/scanner/trigger_scan",
            std::bind(&KsjDriverNode::triggerScanCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
    }

    ~KsjDriverNode()
    {
        if (scanning_) {
            KSJ3D_StopAcquisition(0);
        }
        KSJ3D_UnInitial();
    }

private:

    bool initializeScanner()
    {
        int ret = KSJ3D_Inital();
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_Inital failed: %d", ret);
            return false;
        }
        int device_count = 0;
        ret = KSJ3D_GetDeviceCount(&device_count);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_GetDeviceCount failed: %d", ret);
            return false;
        }
        if (device_count <= 0) {
            RCLCPP_ERROR(this->get_logger(), "No scanner devices found!");
            return false;
        }
        int nType, nSerial;
        unsigned short usFX3, usFPGA;
        ret = KSJ3D_GetDeviceInformation(0, &nType, &nSerial, &usFX3, &usFPGA);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_GetDeviceInformation failed: %d", ret);
            return false;
        }
        int nColMax, nRowMax;
        ret = KSJ3D_GetRoiMax(0, &nColMax, &nRowMax);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_GetRoiMax failed: %d", ret);
            return false;
        }
        ret = KSJ3D_SetRoi(0, 0, 0, nColMax, nRowMax);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_SetRoi failed: %d", ret);
            return false;
        }
        ret = KSJ3D_SetExposureTime(0, EXPOSURE_TIME_MS);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_SetExposureTime failed: %d", ret);
            return false;
        }
        ret = KSJ3D_SetGain(0, GAIN);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_SetGain failed: %d", ret);
            return false;
        }
        ret = KSJ3D_Set3DLaserLineBrightnessThreshold(0, BRIGHTNESS_THRESHOLD);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_Set3DLaserLineBrightnessThreshold failed: %d", ret);
            return false;
        }
        ret = KSJ3D_Set3DLaserLineBrightnessLowThreshold(0, BRIGHTNESS_LOW_THRESHOLD);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_Set3DLaserLineBrightnessLowThreshold failed: %d", ret);
            return false;
        }
        ret = KSJ3D_Set3DLaserLineWidth(0, LASER_LINE_WIDTH);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_Set3DLaserLineWidth failed: %d", ret);
            return false;
        }
        ret = KSJ3D_SetStartTrigger(0, STS_INPUT_0, false, TEM_RISING_EDGE);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_SetStartTrigger failed: %d", ret);
            return false;
        }
        ret = KSJ3D_SetDataTriggerMode(0, DTM_INTERNAL);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_SetDataTriggerMode failed: %d", ret);
            return false;
        }
        ret = KSJ3D_LaserModeSet(0, LM_FLASH);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_LaserModeSet failed: %d", ret);
            return false;
        }
        float fResY;
        ret = KSJ3D_SetYResolution(0, Y_RESOLUTION_MM);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_SetYResolution failed: %d", ret);
            return false;
        }
        ret = KSJ3D_GetYResolution(0, &fResY);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_GetYResolution failed: %d", ret);
            return false;
        }
        if (fResY <= 0.0f) {
            RCLCPP_ERROR(this->get_logger(), "Invalid scanner Y resolution: %.6f", fResY);
            return false;
        }
        double scan_duration_s = Y_TRAVEL_DISTANCE_MM / (ROBOT_SPEED_M_S * 1000.0);
        int num_profiles = static_cast<int>(Y_TRAVEL_DISTANCE_MM / fResY);
        int scan_rate_hz = static_cast<int>(num_profiles / scan_duration_s);
        scan_rate_hz = std::min(scan_rate_hz, MAX_SCAN_RATE_HZ);
        if (scan_rate_hz == MAX_SCAN_RATE_HZ) {
            num_profiles = static_cast<int>(scan_duration_s * scan_rate_hz);
        }
        ret = KSJ3D_SetDataTriggerInternalFrequency(0, scan_rate_hz);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_SetDataTriggerInternalFrequency failed: %d", ret);
            return false;
        }
        ret = KSJ3D_SetMaxNumberOfProfilesToCapture(0, num_profiles);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_SetMaxNumberOfProfilesToCapture failed: %d", ret);
            return false;
        }
        ret = KSJ3D_SetDataFormat(0, KSJ3D_DF_UNIFORMX_INDENSITY);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_SetDataFormat failed: %d", ret);
            return false;
        }
        float fMin, fMax;
        ret = KSJ3D_GetUniformXResolutionRange(0, &fMin, &fMax);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_GetUniformXResolutionRange failed: %d", ret);
            return false;
        }
        ret = KSJ3D_SetUniformXResolution(0, fMin);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_SetUniformXResolution failed: %d", ret);
            return false;
        }
        ret = KSJ3D_GetUniformXResolution(0, &g_fXStart, &g_fXRes, &g_nUniformWidth);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_GetUniformXResolution failed: %d", ret);
            return false;
        }
        float x_width_mm = g_nUniformWidth * g_fXRes;
        RCLCPP_INFO(this->get_logger(), "[SCANNER] Scan dimensions:");
        RCLCPP_INFO(this->get_logger(), "  X (width):  %.1f mm  (%d pts @ %.4f mm/pt, start=%.2f mm)",
                    x_width_mm, g_nUniformWidth, g_fXRes, g_fXStart);
        RCLCPP_INFO(this->get_logger(), "  Y (travel): %.1f mm  (%d profiles @ %.4f mm/profile)",
                    Y_TRAVEL_DISTANCE_MM, num_profiles, (double)Y_TRAVEL_DISTANCE_MM / num_profiles);
        KSJ3D_RegisterZIDataCB(0, &KsjDriverNode::ZICallbackStatic, this);
        RCLCPP_INFO(this->get_logger(), "[SCANNER] Ready - %dHz, %d profiles, %.1fs scan time", 
                    scan_rate_hz, num_profiles, scan_duration_s);
        return true;
    }

    void triggerScanCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (scanning_) {
            response->success = false;
            response->message = "Scan already in progress";
            return;
        }
        scan_received_ = false;
        scanning_ = true;
        scan_start_time_ = this->now();  // record when arm started moving
        int ret = KSJ3D_StartAcquisition(0);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_StartAcquisition failed: %d", ret);
            response->success = false;
            response->message = "Failed to start acquisition";
            scanning_ = false;
            return;
        }
        // Wait for scan to complete (max 15 seconds for slower scans)
        auto start = this->now();
        while (!scan_received_ && (this->now() - start).seconds() < 15.0) {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        ret = KSJ3D_StopAcquisition(0);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "KSJ3D_StopAcquisition failed: %d", ret);
        }
        scanning_ = false;
        if (scan_received_) {
            response->success = true;
            response->message = "Scan completed successfully";
        } else {
            response->success = false;
            response->message = "Scan timeout - no data received";
        }
    }

    static void ZICallbackStatic(int nIndex, float *z, int nWidth, int nHeight, 
                                 float fYRes, int nTotalLostProfileNum, 
                                 unsigned char* intensity, void *lpContext)
    {
        KsjDriverNode* self = static_cast<KsjDriverNode*>(lpContext);
        if (self) {
            self->ZICallback(nWidth, nHeight, z, fYRes, intensity);
        }
    }

    void ZICallback(int nWidth, int nHeight, float* z, float fYRes, unsigned char* intensity)
    {
        // fYRes is the actual hardware-measured Y spacing in mm — convert to metres
        double y_spacing_m = fYRes / 1000.0;

        // Physical geometry:
        //   scanner X (nWidth pts)       = horizontal laser line    → laser_frame x
        //   scanner Y (nHeight profiles) = arm travel during scan   → laser_frame y
        //   scanner Z (depth, mm)        = downward toward ground   → laser_frame z
        //
        // The message timestamp is set to scan_start_time_ (when the arm began moving
        // for this strip).  This means the TF from laser_frame→base_link is looked up
        // at the arm's START position for this strip, which is a well-defined, consistent
        // reference point across all strips.
        //
        // Profile j=0 corresponds to the arm's start position (TF origin), so
        // y_coords[j] = j * y_spacing_m  (no centering, no negation needed).
        // After the URDF transform the arm-travel direction maps into the world correctly.
        
        // Calculate statistics while processing
        int valid_count = 0;
        int invalid_count = 0;
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();

        // Reconstruct X,Y coordinates for all points
        std::vector<float> x_coords(nWidth * nHeight);
        std::vector<float> y_coords(nWidth * nHeight);

        for (int j = 0; j < nHeight; ++j) {
            for (int i = 0; i < nWidth; i++) {
                int idx = j * nWidth + i;
                x_coords[idx] = (g_fXStart + i * g_fXRes) / 1000.0f;   // laser line → laser_frame x
                y_coords[idx] = static_cast<float>(-(j * y_spacing_m)); // arm travel → laser_frame y (negated for Rx(pi))
                
                float z_val = z[idx];
                
                // Categorize invalid points
                if (!std::isfinite(z_val) || z_val == 0.0f || z_val <= -999.0f) {
                    invalid_count++;
                } else {
                    // Valid point
                    valid_count++;
                    if (z_val < min_z) min_z = z_val;
                    if (z_val > max_z) max_z = z_val;
                }
            }
        }

        int total_points = nWidth * nHeight;
        float valid_percentage = (valid_count * 100.0f) / total_points;
        
        RCLCPP_INFO(this->get_logger(), "[SCANNER] Captured: %.1f%% valid (%d/%d pts)", 
                    valid_percentage, valid_count, total_points);
        if (valid_count > 0) {
            RCLCPP_INFO(this->get_logger(), "[SCANNER] Z range: %.2f mm to %.2f mm (span=%.2f mm)",
                        min_z, max_z, max_z - min_z);
        }

        // Create PointCloud2 message - UNORGANIZED cloud (height=1) with only valid points
        auto msg = sensor_msgs::msg::PointCloud2();
        msg.header.frame_id = "laser_frame";
        // Timestamp = scan start time: the arm was at its start position (TF origin)
        // when this scan began.  Using start time (not midpoint) ensures the TF lookup
        // captures a consistent, well-defined arm position for every strip — unaffected
        // by the diagonal transit the arm makes between strip end and next strip start.
        msg.header.stamp = scan_start_time_;
        msg.height = 1;  // Unorganized cloud
        msg.width = valid_count;  // Only valid points
        msg.is_dense = true;  // No NaN/Inf values

        msg.fields.resize(4);
        msg.fields[0].name = "x";
        msg.fields[0].offset = 0;
        msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[0].count = 1;
        msg.fields[1].name = "y";
        msg.fields[1].offset = 4;
        msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[1].count = 1;
        msg.fields[2].name = "z";
        msg.fields[2].offset = 8;
        msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[2].count = 1;
        msg.fields[3].name = "intensity";
        msg.fields[3].offset = 12;
        msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[3].count = 1;

        msg.is_bigendian = false;
        msg.point_step = 16;
        msg.row_step = msg.point_step * msg.width;
        msg.data.resize(msg.row_step);

        // Pack only valid points
        int point_idx = 0;
        for (int i = 0; i < nWidth * nHeight; i++) {
            float z_val = z[i];
            if (std::isfinite(z_val) && z_val != 0.0f && z_val > -999.0f) {
                float* p = reinterpret_cast<float*>(&msg.data[point_idx * 16]);
                p[0] = x_coords[i];
                p[1] = y_coords[i];
                p[2] = z_val / 1000.0f;  // mm to m
                p[3] = static_cast<float>(intensity[i]);
                point_idx++;
            }
        }

        publisher_->publish(msg);
        scan_received_ = true;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr scan_service_;
    bool scanning_;
    bool scan_received_;
    rclcpp::Time scan_start_time_;  // time when arm started moving (used as cloud timestamp for TF lookup)
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KsjDriverNode>());
    rclcpp::shutdown();
    return 0;
}
