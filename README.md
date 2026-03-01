# Autonomous Asphalt Crack Detection and Repair System

## Project Overview

This project implements an automated robotic system for detecting and repairing cracks in asphalt surfaces using a Dobot robot arm equipped with a 3D laser scanner. The system combines computer vision, 3D scanning, and robotic manipulation to autonomously identify, scan, and repair pavement cracks.

### Key Capabilities
- **3D Laser Scanning**: High-resolution 3D point cloud acquisition of crack surfaces
- **AI-Powered Detection**: Deep learning-based crack segmentation using U-Net
- **Robotic Automation**: Precise robot arm control for scanning and repair operations
- **Multi-Strip Scanning**: Automatic merging of multiple scans for long crack coverage
- **Real-time Visualization**: RViz2 integration for monitoring and debugging

---

## System Architecture

### Hardware Components

1. **Dobot CR-Series Robot Arm**
   - 6-axis collaborative robot
   - Payload capacity for scanner and repair tools
   - Precise positioning for scan alignment

2. **KSJ UC3D230ED 3D Laser Scanner**
   - Model: UC3D230ED-800x600-R
   - Resolution: 1936 x 340 pixels
   - Scanning frequency: 100 Hz
   - Working distance: 20-50 cm
   - Laser wavelength: Red (visible)

3. **Computing Platform**
   - Fedora Linux with ROS2 Humble
   - Distrobox containerization for ROS2 environment
   - GPU acceleration (optional for crack detection)

### Software Stack

```
┌─────────────────────────────────────────────────────────┐
│                    Application Layer                    │
│  - Crack Repair Planning                                │
│  - Multi-Scan Coordination                              │
│  - Path Planning & Execution                            │
└─────────────────────────────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────┐
│                   ROS2 Pipeline Layer                   │
│                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │   Laser      │  │    Crack     │  │    Robot     │ │
│  │   Scanner    │  │  Detection   │  │     Arm      │ │
│  │   Nodes      │  │     Node     │  │   Control    │ │
│  └──────────────┘  └──────────────┘  └──────────────┘ │
│         │                  │                  │         │
│         └──────────────────┴──────────────────┘         │
│                          │                               │
│                   TF2 Transforms                         │
│            (Coordinate Frame Management)                 │
└─────────────────────────────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────┐
│                  Hardware Interface Layer                │
│  - KSJ Scanner SDK (libKSJApi3D.so)                     │
│  - Dobot Robot SDK                                       │
│  - USB/Network Communication                             │
└─────────────────────────────────────────────────────────┘
```

---

## ROS2 Pipeline

### Package Structure

```
src/
├── laser_scanner/              # 3D laser scanning package
│   ├── laser_driver            # Scanner hardware interface
│   └── scan_accumulator        # Multi-scan merging
│
├── crack_detection_ros2/       # AI-based crack detection
│   └── crack_detector_node     # U-Net inference node
│
├── robot_arm/                  # Robot arm control
│   └── arm_controller          # Motion planning & control
│
├── crack_repair/               # Crack repair coordination
│   └── repair_planner          # High-level task planning
│
└── DOBOT_6Axis_ROS2_V4/       # Dobot driver (vendor provided)
```

### Data Flow Pipeline

```
┌─────────────────────┐
│  Camera/Scanner     │
│   /camera/image     │ ──┐
└─────────────────────┘   │
                          │
┌─────────────────────┐   │    ┌──────────────────────┐
│  Laser Scanner      │   ├───▶│  Crack Detection     │
│  /scanner/trigger   │   │    │  (U-Net Model)       │
│  /scanner/single    │───┘    │  Binary Mask Output  │
│       _scan         │        └──────────────────────┘
└─────────────────────┘                 │
         │                              ▼
         │                    ┌──────────────────────┐
         │                    │  Crack Analysis      │
         │                    │  - Location          │
         │                    │  - Dimensions        │
         │                    │  - Repair Strategy   │
         │                    └──────────────────────┘
         │                              │
         ▼                              ▼
┌─────────────────────┐        ┌──────────────────────┐
│  Scan Accumulator   │        │   Robot Planner      │
│  - TF2 Transform    │◀───────│  - Path generation   │
│  - Point cloud      │        │  - Scan positions    │
│    merging          │        │  - Motion commands   │
│  /scanner/merged    │        └──────────────────────┘
└─────────────────────┘                 │
         │                              │
         ▼                              ▼
┌─────────────────────┐        ┌──────────────────────┐
│  Point Cloud        │        │   Robot Arm          │
│  Storage (PCD)      │        │   /robot/move_to     │
│  /scanner/save      │        │   /robot/execute     │
└─────────────────────┘        └──────────────────────┘
```

### Key ROS2 Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Raw camera feed |
| `/scanner/single_scan` | `sensor_msgs/PointCloud2` | Single laser scan strip |
| `/scanner/merged_cloud` | `sensor_msgs/PointCloud2` | Accumulated point cloud |
| `/crack_detection/binary_mask` | `sensor_msgs/Image` | Detected crack mask (white=crack, black=background) |
| `/robot/joint_states` | `sensor_msgs/JointState` | Robot joint positions |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |

### Key ROS2 Services

| Service | Type | Description |
|---------|------|-------------|
| `/scanner/trigger_scan` | `std_srvs/Trigger` | Trigger single scan capture |
| `/scanner/save_merged_cloud` | `std_srvs/Trigger` | Save accumulated point cloud to PCD |
| `/scanner/clear_scans` | `std_srvs/Empty` | Clear accumulated scans |
| `/scanner/get_scan_count` | `std_srvs/Trigger` | Get number of accumulated scans |
| `/robot/move_to_position` | Custom | Move robot to specified pose |

---

## Workflow: Crack Scanning & Detection

### Phase 1: Multi-Strip 3D Scanning

The system scans long cracks by dividing them into multiple overlapping vertical strips:

```
Crack Surface (top view):
═════════════════════════════════════════════
        ║                                    
    [Strip 1]  [Strip 2]  [Strip 3]  [Strip 4]
       ║         ║         ║         ║        
       ▼         ▼         ▼         ▼        
    ████████  ████████  ████████  ████████
         ████████  ████████  ████████        (10-20% overlap)
              
Total Coverage: Merged into single point cloud
```

**Process:**
1. Robot positions scanner at Strip 1 (leftmost position)
2. Service call triggers laser scan while robot moves slowly (~2cm)
3. Scanner captures 200 profiles at 100Hz over 2 seconds
4. `scan_accumulator` transforms scan to `base_link` frame using TF2
5. Robot moves to Strip 2 position (with overlap)
6. Repeat until entire crack is covered
7. Save merged point cloud

### Phase 2: Crack Detection

1. Camera captures 2D image of scanned area
2. Image fed to U-Net crack detection model
3. Binary mask output (cracks = white, background = black)
4. Mask published to `/crack_detection/binary_mask`

### Phase 3: Repair Planning

1. Combine 3D point cloud + 2D crack mask
2. Extract crack geometry (length, width, depth)
3. Generate repair toolpath
4. Execute robot motion for repair material application

---

## Component Details

### 1. Laser Scanner Package (`laser_scanner`)

**Purpose:** Interface with KSJ 3D laser scanner hardware and manage scan acquisition

**Nodes:**

#### `laser_driver`
- **Executable:** `ros2 run laser_scanner laser_driver`
- **Functionality:**
  - Initializes KSJ UC3D230ED scanner
  - Configures scanning parameters (exposure, gain, frequency)
  - Captures single scan strips on service trigger
  - Publishes PointCloud2 with X, Y, Z, intensity data
  - Uses internal 100Hz trigger mode
  
- **Key Parameters:**
  ```yaml
  exposure_time: 0.5 ms        # Optimized for dark asphalt
  gain: 10                     # Increased sensitivity
  brightness_threshold: 10     # Lower for dark surfaces
  profiles_per_scan: 200       # 2 seconds @ 100Hz
  y_resolution: 0.1 mm         # Spacing between profiles
  ```

#### `scan_accumulator`
- **Executable:** `ros2 run laser_scanner scan_accumulator`
- **Functionality:**
  - Subscribes to `/scanner/single_scan`
  - Transforms each scan to common reference frame (`base_link`)
  - Accumulates scans into merged point cloud
  - Publishes live preview to `/scanner/merged_cloud`
  - Saves final merged cloud to PCD file
  
- **Key Parameters:**
  ```yaml
  target_frame: "base_link"
  output_directory: "/tmp"
  auto_publish_interval_ms: 1000
  ```

**Coordinate System:**
- **X-axis:** Across laser line (scanner width)
- **Y-axis:** Motion direction (scan progress)
- **Z-axis:** Height/depth from scanner

---

### 2. Crack Detection Package (`crack_detection_ros2`)

**Purpose:** Real-time crack segmentation using deep learning

**Node:** `crack_detector_node`
- **Executable:** `ros2 run crack_detection_ros2 crack_detector`
- **Model:** U-Net architecture trained on asphalt crack dataset
- **Input:** RGB camera images (any resolution)
- **Output:** Binary mask (255=crack, 0=background)
- **Processing Rate:** Configurable (default 10 Hz)

**Key Features:**
- Pre-trained model (`unet_crack_detector.keras`)
- Confidence threshold tuning (default 0.5)
- GPU acceleration support (TensorFlow)
- Thread-safe processing with mutex locks

---

### 3. Robot Arm Control

**Integration:** Uses Dobot's ROS2 driver (`DOBOT_6Axis_ROS2_V4`)

**Key Functionality:**
- Forward/inverse kinematics
- Trajectory planning
- Collision avoidance
- Joint/Cartesian control modes
- TF2 frame broadcasting

---

## System Requirements

### Software Dependencies

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Point Cloud Library
sudo apt install libpcl-dev pcl-tools

# ROS2 PCL integration
sudo apt install ros-humble-pcl-conversions ros-humble-pcl-ros

# TensorFlow (for crack detection)
pip install tensorflow>=2.8.0

# OpenCV
pip install opencv-python>=4.5.0

# Additional Python packages
pip install numpy scikit-learn
```

### Hardware Requirements
- **CPU:** 4+ cores recommended
- **RAM:** 8GB minimum, 16GB recommended
- **GPU:** Optional (NVIDIA with CUDA for faster inference)
- **USB:** USB 3.0 port for laser scanner
- **Storage:** ~10GB for dependencies + point cloud data

---

## Getting Started

### 1. Build the Workspace

```bash
cd /home/juyoungoh/University/CATS/DOBOT/Dobot_Robot_Arm

# Source ROS2
source /opt/ros/humble/setup.bash

# Build all packages
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### 2. Configure USB Permissions (First Time Only)

```bash
# Add udev rule for KSJ scanner (vendor ID: 0816)
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0816", MODE="0666"' | sudo tee /etc/udev/rules.d/99-ksj-scanner.rules

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout group
sudo usermod -aG dialout $USER

# Log out and back in for group changes to take effect
```

### 3. Launch the System

**Terminal 1: Laser Scanner Driver**
```bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PWD/src/laser_scanner/include/KSJApi.bin/x64
ros2 run laser_scanner laser_driver
```

**Terminal 2: Scan Accumulator**
```bash
source install/setup.bash
ros2 run laser_scanner scan_accumulator \
  --ros-args \
  -p target_frame:=base_link \
  -p output_directory:=/tmp
```

**Terminal 3: Crack Detection (Optional)**
```bash
source install/setup.bash
ros2 run crack_detection_ros2 crack_detector
```

**Terminal 4: RViz Visualization**
```bash
source install/setup.bash
ros2 run rviz2 rviz2
```

### 4. Perform a Scan

```bash
# Terminal 5: Control
source install/setup.bash

# Clear previous scans
ros2 service call /scanner/clear_scans std_srvs/srv/Empty

# Position 1: Move robot, then trigger scan
ros2 service call /robot/move_to_position [...]
ros2 service call /scanner/trigger_scan std_srvs/srv/Trigger

# Position 2: Move robot with overlap
ros2 service call /robot/move_to_position [...]
ros2 service call /scanner/trigger_scan std_srvs/srv/Trigger

# Repeat for all positions...

# Save merged point cloud
ros2 service call /scanner/save_merged_cloud std_srvs/srv/Trigger
```

---

## Configuration & Tuning

### Scanner Parameters (for Dark Asphalt)

Located in `src/laser_scanner/src/laser_driver.cpp`:

```cpp
KSJ3D_SetExposureTime(0, 0.5);              // Longer exposure for dark surfaces
KSJ3D_SetGain(0, 10);                       // Higher gain for better sensitivity
KSJ3D_Set3DLaserLineBrightnessThreshold(0, 10);  // Lower threshold
KSJ3D_Set3DLaserLineWidth(0, 30);          // Less strict line width
```

### Scan Coverage Planning

```cpp
// Each scan covers approximately:
// Width: ~19cm (laser FOV)
// Length: num_profiles × y_resolution
//       = 200 × 0.1mm = 20mm (2cm)

// For overlapping scans:
scan_width = 0.019;      // 19mm effective width
overlap = 0.003;         // 3mm overlap (15%)
step_size = 0.016;       // 16mm between scan positions
```

### Crack Detection Tuning

```python
# In crack_detector_node.py
confidence_threshold = 0.5  # Adjust sensitivity (0.3-0.7)
publish_rate = 10.0         # Processing frequency (Hz)
```

---

## Troubleshooting

### Scanner Issues

**Problem:** "No devices found" or "Permission denied"
- **Solution:** Check USB connection, verify udev rules, try with `sudo`

**Problem:** All NaN values in point cloud
- **Solution:** 
  - Ensure object is moving during scan (2cm over 2 seconds)
  - Check laser line is visible on surface
  - Verify object distance (25-35cm optimal)
  - Use lighter colored test surface initially

**Problem:** Scanner disconnects repeatedly
- **Solution:** 
  ```bash
  # Disable USB autosuspend
  sudo sh -c 'echo -1 > /sys/module/usbcore/parameters/autosuspend'
  ```

### ROS2 Issues

**Problem:** Scans not accumulating
- **Solution:** Check TF2 transforms are being published:
  ```bash
  ros2 run tf2_ros tf2_echo base_link laser_frame
  ```

**Problem:** Point clouds not aligned
- **Solution:** Verify robot TF tree, ensure proper frame_id in scanner

---

## Project Structure

```
Dobot_Robot_Arm/
├── README.md                          # This file
├── LASER_SCANNER_GUIDE.md            # Detailed scanner API documentation
├── LASER_SCANNER_QUICKSTART.md       # Quick start guide
├── install.sh                         # Installation script
├── setup.sh                          # Environment setup
│
├── src/                              # ROS2 packages
│   ├── laser_scanner/               # 3D scanning
│   ├── crack_detection_ros2/        # AI detection
│   ├── robot_arm/                   # Robot control
│   ├── crack_repair/                # High-level planning
│   └── DOBOT_6Axis_ROS2_V4/        # Dobot driver
│
├── Development-Linux_x64/           # KSJ Scanner SDK
│   ├── KSJApi.inc/                 # Headers
│   ├── KSJApi.bin/                 # Shared libraries
│   ├── KSJShow3D/                  # GUI tool
│   └── Samples/                    # Example code
│
└── CrackDetection/                 # ML models
    ├── crack_detect.py             # Training script
    └── unet_crack_detector.keras   # Pre-trained model
```

---

## Future Enhancements

1. **Encoder-Triggered Scanning**
   - Sync laser profiles directly with robot encoder
   - Ensures consistent point spacing regardless of speed variations

2. **Automatic Point Cloud Registration**
   - ICP/NDT algorithms for precise scan alignment
   - Reduces dependency on perfect robot positioning

3. **Real-time Crack Analysis**
   - 3D crack width/depth measurement
   - Volume estimation for repair material calculation

4. **Closed-Loop Repair**
   - Post-repair scanning for quality verification
   - Adaptive filling based on actual vs. expected results

5. **Multi-Robot Coordination**
   - Parallel scanning for faster coverage
   - Fleet management for large-area operations

---

## References

- **KSJ Scanner Documentation:** `Development-Linux_x64/Doc/`
- **ROS2 Humble Documentation:** https://docs.ros.org/en/humble/
- **PCL Documentation:** https://pointclouds.org/
- **Dobot SDK:** `src/DOBOT_6Axis_ROS2_V4/`

---

## License

[Specify your license here]

## Contributors

- Juyoung Oh (@ohjy1006kenneth)
- CATS Research Group

## Contact

For questions or issues, please open an issue on the GitHub repository.
