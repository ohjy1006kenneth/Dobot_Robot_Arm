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
                # Fit a line to the largest white contour
                [vx, vy, x, y] = cv2.fitLine(largest_contour, cv2.DIST_L2, 0, 0.01, 0.01)

                # Calculate points for the fitted line
                left_y = int((-x * vy / vx) + y)
                right_y = int(((width - x) * vy / vx) + y)

                if math.isnan(left_y) or math.isnan(right_y):
                    self.get_logger().warn("NaN detected in line fit points — skipping publish")
                    return  # skip publishing this cycle

                pt1 = (0, int(left_y + crop_y_start))
                pt2 = (width - 1, int(right_y + crop_y_start))

                # Draw red fitted line
                cv2.line(cv_image, pt1, pt2, (0, 0, 255), 2)

                # Calculate centroid x of the contour
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])

                    # Error is distance from image center
                    error = cx - (width // 2)
                    self.publisher_.publish(Int32(data=error))
                    self.get_logger().info(f"Error deviation: {error}")
        else:
            self.get_logger().info("Error deviation: no line detected")

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
