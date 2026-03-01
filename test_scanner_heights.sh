#!/bin/bash
# Test script to check scanner behavior at different heights

echo "================================"
echo "Laser Scanner Height Test"
echo "================================"
echo ""
echo "This script will help you test the scanner at different heights"
echo ""
echo "Instructions:"
echo "1. Position the scanner at the specified height"
echo "2. Press ENTER to trigger a scan"
echo "3. Note the 'Valid points %' from the log"
echo ""

source ~/Dobot_Robot_Arm/install/setup.bash

heights=(100 150 200 250 300 350 400 500 600 800 1000)

echo "Test Heights: ${heights[@]} mm"
echo ""

for height in "${heights[@]}"; do
    echo "================================"
    echo "Test Height: ${height}mm"
    echo "================================"
    echo "Position scanner at ${height}mm from ground and press ENTER"
    read -p ""
    
    echo "Triggering scan..."
    ros2 service call /scanner/trigger_scan std_srvs/srv/Trigger
    
    echo ""
    echo "Check the laser_driver terminal for:"
    echo "  - Valid: X/Y (Z.Z%)"
    echo "  - Z range: min - max mm"
    echo ""
    echo "Record these values for height ${height}mm"
    echo ""
    read -p "Press ENTER to continue to next height..."
    echo ""
done

echo "================================"
echo "Test Complete!"
echo "================================"
echo "Review your recorded values to find optimal working distance"
