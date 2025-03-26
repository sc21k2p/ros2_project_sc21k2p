# Exercise 3 - If green object is detected, and above a certain size, then send a message (print or use lab2)

import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from math import sin, cos


class multiTaskRobot(Node):
    def __init__(self):
        super().__init__('cI')
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.laser_subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.subscription
        self.sensitivity = 10
        
        self.green_identified = False
        self.red_identified = False
        self.blue_identified = False

        self.moveForwardsFlag = False
        self.stopFlag = False
        
        self.rate = self.create_rate(10)
        
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def laser_callback(self, msg):
        position = msg.ranges[0]
        if position <= 1.00:
            # self.moveForwardsFlag = False
            # print("moveForwards deactivated")
            self.stopFlag = True
            print("Stop activated")
            


    def callback(self, data):

        # Convert the received image into a opencv image
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError:
            print("An error has occurred with displaying the image")
        
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        # Convert the rgb image into a hsv image
        
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        
        # red
        hsv_red_lower_1 = np.array([0, 100, 100])
        hsv_red_upper_1 = np.array([0+self.sensitivity, 255, 255])
        
        hsv_red_lower_2 = np.array([180 - self.sensitivity, 100, 100])
        hsv_red_upper_2 = np.array([180, 255, 255])

        # Filter out everything but a particular colour using the cv2.inRange() method
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
        
        red_mask_1 = cv2.inRange(hsv_image, hsv_red_lower_1, hsv_red_upper_1)
        red_mask_2 = cv2.inRange(hsv_image, hsv_red_lower_2, hsv_red_upper_2)
        
        # Apply the mask to the original image using the cv2.bitwise_and() method
        red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)
        
        # rg_mask = cv2.bitwise_or(red_mask, blue_mask)

        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        # filtered_image = cv2.bitwise_and(image, image, mask=rg_mask)
        
        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        
        # RED
        red_contours, _ = cv2.findContours(red_mask,mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)
        
        if len(red_contours) > 0:
            c = max(red_contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

            
            if cv2.contourArea(c) > 1:
                self.red_identified = True
                print("Red Identified", cv2.contourArea(c))
        
        # GREEN
        green_contours, _ = cv2.findContours(green_mask,mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)
        
        if len(green_contours) > 0:
            c = max(green_contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

            
            if cv2.contourArea(c) > 1:
                self.green_identified = True
                print("Green Identified", cv2.contourArea(c))
                
        # BLUE
        blue_contours, _ = cv2.findContours(blue_mask,mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)
        
        if len(blue_contours) > 0:
            c = max(blue_contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            
            if cv2.contourArea(c) > 1:
                self.blue_identified = True
                print("Blue Identified", cv2.contourArea(c))
                
                self.moveForwardsFlag = True
                print("moveForwards activated")
                
                
                # if cv2.contourArea(c) > 175000:
                #     self.moveForwardsFlag = False
                #     print("moveForwards deactivated")
                #     self.stopFlag = True
                #     print("Stop activated")
                    # self.stop()
                
                # elif cv2.contourArea(c) <= 175000:
                #     self.moveForwardsFlag = True
                #     print("moveForwards activated")
                    # self.walk_forward()

                # draw a circle on the contour you're identifying
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
                # (x, y), radius = cv2.minEnclosingCircle(c)

                # cv2.circle(image,(x, y),radius,colour,thickness)
  
                # Then alter the values of any flags
                
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(3)
        
    def walk_forward(self):
        #Use what you learnt in lab 3 to make the robot move forwards
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2
        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def stop(self):
        # Use what you learnt in lab 3 to make the robot stop
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0
        self.publisher.publish(desired_velocity)
        
    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.action_client.wait_for_server()
        # self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future = self.action_client.send_goal(goal_msg)
        # self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # NOTE: if you want, you can use the feedback while the robot is moving.
        #       uncomment to suit your need.

        ## Access the current pose
        #current_pose = feedback_msg.feedback.current_pose
        #position = current_pose.pose.position
        #orientation = current_pose.pose.orientation

        ## Access other feedback fields
        #navigation_time = feedback_msg.feedback.navigation_time
        #distance_remaining = feedback_msg.feedback.distance_remaining

        ## Print or process the feedback data
        #self.get_logger().info(f'Current Pose: [x: {position.x}, y: {position.y}, z: {position.z}]')
        #self.get_logger().info(f'Distance Remaining: {distance_remaining}')
    
        





# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args = None):
    def signal_handler(sig, frame):
        mTR.stop()
        rclpy.shutdown()

    
    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=args)
   
    mTR = multiTaskRobot()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(mTR,), daemon=True)
    thread.start()
    
      # example coordinates
    
    
    
    try:
        while rclpy.ok():
            if mTR.blue_identified == True:
                if mTR.stopFlag == True:
                    mTR.stop()
                elif mTR.moveForwardsFlag == True:
                    mTR.walk_forward()
                
            else:
                mTR.send_goal(-0.75, -3.11, -0.00143)
                mTR.send_goal(0.9, -6.01, 0.00247)
                mTR.send_goal(-4.32, -1.21, 0.00247)
                mTR.send_goal(-5.55, -9.38, 0.00256)
                
                
    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()


# Check if the node is executing in the main path
if __name__ == '__main__':
    main()