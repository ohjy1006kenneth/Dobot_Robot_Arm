#include <memory>
#include <limits>
#include <cmath>
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

        // Service to trigger a single scan
        scan_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/scanner/trigger_scan",
            std::bind(&KsjDriverNode::triggerScanCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));

        initializeScanner();
    }

    ~KsjDriverNode()
    {
        if (scanning_) {
            KSJ3D_StopAcquisition(0);
        }
        KSJ3D_UnInitial();
    }

private:

    void initializeScanner()
    {
        KSJ3D_Inital();
        
        int device_count = 0;
        KSJ3D_GetDeviceCount(&device_count);
        
        if (device_count <= 0) {
            RCLCPP_ERROR(this->get_logger(), "No scanner devices found!");
            return;
        }

        // Get device info
        int nType, nSerial;
        unsigned short usFX3, usFPGA;
        KSJ3D_GetDeviceInformation(0, &nType, &nSerial, &usFX3, &usFPGA);

        // Configure scanner
        int nColMax, nRowMax;
        KSJ3D_GetRoiMax(0, &nColMax, &nRowMax);
        KSJ3D_SetRoi(0, 0, 0, nColMax, nRowMax);
        
        KSJ3D_SetExposureTime(0, EXPOSURE_TIME_MS);
        KSJ3D_SetGain(0, GAIN);
        KSJ3D_Set3DLaserLineBrightnessThreshold(0, BRIGHTNESS_THRESHOLD);
        KSJ3D_Set3DLaserLineBrightnessLowThreshold(0, BRIGHTNESS_LOW_THRESHOLD);
        KSJ3D_Set3DLaserLineWidth(0, LASER_LINE_WIDTH);
        
        KSJ3D_SetStartTrigger(0, STS_INPUT_0, 0, TEM_RISING_EDGE);
        KSJ3D_SetDataTriggerMode(0, DTM_INTERNAL);
        KSJ3D_LaserModeSet(0, LM_FLASH);
        
        // Calculate optimal scan parameters
        double robot_speed_mm_s = ROBOT_SPEED_M_S * 1000.0;
        double scan_duration_s = Y_TRAVEL_DISTANCE_MM / robot_speed_mm_s;
        int scan_rate_hz = static_cast<int>(TARGET_PROFILES / scan_duration_s);
        scan_rate_hz = std::min(scan_rate_hz, MAX_SCAN_RATE_HZ);
        int num_profiles = static_cast<int>(scan_duration_s * scan_rate_hz);
        
        KSJ3D_SetDataTriggerInternalFrequency(0, scan_rate_hz);
        KSJ3D_SetMaxNumberOfProfilesToCapture(0, num_profiles);
        
        // Set data format to Mode 4
        KSJ3D_SetDataFormat(0, KSJ3D_DF_UNIFORMX_INDENSITY);
        
        float fMin, fMax;
        KSJ3D_GetUniformXResolutionRange(0, &fMin, &fMax);
        KSJ3D_SetUniformXResolution(0, fMin);  // Use minimum value for MAXIMUM X resolution
        KSJ3D_GetUniformXResolution(0, &g_fXStart, &g_fXRes, &g_nUniformWidth);

        // Register callback
        KSJ3D_RegisterZIDataCB(0, &KsjDriverNode::ZICallbackStatic, this);
        
        RCLCPP_INFO(this->get_logger(), "[SCANNER] Ready - %dHz, %d profiles, %.1fs scan time", 
                    scan_rate_hz, num_profiles, scan_duration_s);
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

        int ret = KSJ3D_StartAcquisition(0);
        if (ret != 0) {
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

        KSJ3D_StopAcquisition(0);
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
        // Y spacing based on actual travel distance
        double y_spacing_m = (Y_TRAVEL_DISTANCE_MM / 1000.0) / std::max(1, nHeight);
        
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
                x_coords[idx] = (g_fXStart + i * g_fXRes) / 1000.0f;
                y_coords[idx] = j * y_spacing_m;
                
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

        // Create PointCloud2 message - UNORGANIZED cloud (height=1) with only valid points
        auto msg = sensor_msgs::msg::PointCloud2();
        msg.header.frame_id = "laser_frame";
        msg.header.stamp = this->get_clock()->now();
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
        msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
        msg.fields[3].count = 1;

        msg.is_bigendian = false;
        msg.point_step = 13;
        msg.row_step = msg.point_step * msg.width;
        msg.data.resize(msg.row_step);

        // Pack only valid points
        int point_idx = 0;
        for (int i = 0; i < nWidth * nHeight; i++) {
            float z_val = z[i];
            if (std::isfinite(z_val) && z_val != 0.0f && z_val > -999.0f) {
                float* p = reinterpret_cast<float*>(&msg.data[point_idx * 13]);
                p[0] = x_coords[i];
                p[1] = y_coords[i];
                p[2] = z_val / 1000.0f;  // mm to m
                msg.data[point_idx * 13 + 12] = intensity[i];
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
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KsjDriverNode>());
    rclcpp::shutdown();
    return 0;
}