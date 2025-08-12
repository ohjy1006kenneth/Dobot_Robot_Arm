import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32
import cv2
import math
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Int32, '/line_error', 10)
        self.crack_pub = self.create_publisher(Int32, '/crack_angle', 10)

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV Image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = cv_image.shape

        # Convert to grayscale and threshold (keep white line)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)  # Keep white

        # Focus on the bottom 25% of the image
        crop_y_start = int(0.75 * height)
        crop = binary[crop_y_start:, :]

        # Find contours
        contours, _ = cv2.findContours(crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if len(largest_contour) >= 2:
                [vx, vy, x, y] = cv2.fitLine(largest_contour, cv2.DIST_L2, 0, 0.01, 0.01)

                left_y = (-x * vy / vx) + y
                right_y = ((width - x) * vy / vx) + y

                pt1_white = (0, int(left_y + crop_y_start))
                pt2_white = (width - 1, int(right_y + crop_y_start))

                # NaN and Inf check:
                # if not any(math.isnan(coord) or math.isinf(coord) for coord in [pt1_white[1], pt2_white[1]]):
                #     cv2.line(cv_image, pt1_white, pt2_white, (255, 0, 0), 2)  # Blue line

                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    error = cx - (width // 2)
                    # self.publisher_.publish(Int32(data=error))
                    
        # Detect Crack
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        blue_crop = blue_mask[crop_y_start:, :]
        blue_contours, _ = cv2.findContours(blue_crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_blue_contour_area = 500  # Tune this threshold based on your camera feed & resolution

        if blue_contours:
            # Filter contours by area
            large_blue_contours = [cnt for cnt in blue_contours if cv2.contourArea(cnt) > min_blue_contour_area]

            if large_blue_contours:
                largest_blue = max(large_blue_contours, key=cv2.contourArea)

                if len(largest_blue) >= 2:
                    try:
                        [vx, vy, x, y] = cv2.fitLine(largest_blue, cv2.DIST_L2, 0, 0.01, 0.01)

                        angle_rad = math.atan2(vy, vx)
                        angle_deg = int(math.degrees(angle_rad))

                        left_y = (-x * vy / vx) + y
                        right_y = ((cv_image.shape[1] - x) * vy / vx) + y

                        pt1_blue = (0, int(left_y + crop_y_start))
                        pt2_blue = (cv_image.shape[1] - 1, int(right_y + crop_y_start))

                        # NaN and Inf check:
                        if any(math.isnan(coord) or math.isinf(coord) for coord in [pt1_blue[1], pt2_blue[1]]):
                            raise ValueError("NaN or Inf detected in blue line points")

                        cv2.line(cv_image, pt1_blue, pt2_blue, (0, 0, 255), 2)  # Red line
                        self.crack_pub.publish(Int32(data=angle_deg))
                        self.get_logger().info(f"Blue line detected: angle {angle_deg}")

                    except Exception as e:
                        self.get_logger().warn(f"Blue line fit error: {e}")
                        self.crack_pub.publish(Int32(data=9999))
                else:
                    self.get_logger().info("Largest blue contour too small or insufficient points")
                    self.crack_pub.publish(Int32(data=9999))
                    self.publisher_.publish(Int32(data=error))
                    if not any(math.isnan(coord) or math.isinf(coord) for coord in [pt1_white[1], pt2_white[1]]):
                        cv2.line(cv_image, pt1_white, pt2_white, (255, 0, 0), 2)  # Blue line
            else:
                self.get_logger().info("No blue contours above area threshold")
                self.crack_pub.publish(Int32(data=9999))
                self.publisher_.publish(Int32(data=error))
                if not any(math.isnan(coord) or math.isinf(coord) for coord in [pt1_white[1], pt2_white[1]]):
                    cv2.line(cv_image, pt1_white, pt2_white, (255, 0, 0), 2)  # Blue line
        else:
            self.get_logger().info("No blue contours found")
            self.crack_pub.publish(Int32(data=9999))
            self.publisher_.publish(Int32(data=error))
            if not any(math.isnan(coord) or math.isinf(coord) for coord in [pt1_white[1], pt2_white[1]]):
                cv2.line(cv_image, pt1_white, pt2_white, (255, 0, 0), 2)  # Blue line

        # Show image
        cv2.imshow("Line Follower with Red Line", cv_image)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
