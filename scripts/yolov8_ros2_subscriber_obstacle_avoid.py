#!/usr/bin/env python3

import cv2
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()

from yolov8_msgs.msg import Yolov8Inference

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import math

# from nav2_simple_commander.robot_navigator import BasicNavigator
# nav = BasicNavigator()
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile


class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10)
        self.subscription 

    def camera_callback(self, data):
        global img
        img = bridge.imgmsg_to_cv2(data, "bgr8")

class Yolo_subscriber(Node):

    def __init__(self):
        super().__init__('yolo_subscriber')

        #code from yolov8 inference subscribing and img_publishing
        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10)
        self.subscription 

        self.cnt = 0

        self.img_pub = self.create_publisher(Image, "/inference_result_cv2", 1)#if you subscribe to this, you can recreate the image with the bounding box on another node

        #####################################
        #code from obstacle avoid

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.linear_velocity = 0.0  # unit: m/s
        self.angular_velocity = 0.0  # unit: m/s
        self.scan_ranges = []
        self.init_scan_state = False  # To get the initial scan data at the beginning
        self.object_detected = False
        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos)

        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(
            0.010,  # unit: s
            self.update_callback)

        self.get_logger().info("Turtlebot3 obstacle detection node has been initialised.")


    def yolo_callback(self, data):
        global img
        self.object_detected = False

        for r in data.yolov8_inference:
            self.object_detected = True
            class_name = r.class_name
            top= r.top#x coord of top left point
            left= r.left#y coord of top left point
            bottom= r.bottom#x coord of bottom right point
            right= r.right#y coord of bottom right point
            self.left_bound = top
            self.right_bound = bottom
            yolo_subscriber.get_logger().info(f"{self.cnt} {class_name} : {top}, {left}, {bottom}, {right}")
            cv2.rectangle(img, (top, left), (bottom, right), (255, 255, 0))
            h, w, c = img.shape
            self.image_width = w
            self.cnt += 1

        self.cnt = 0
        img_msg = bridge.cv2_to_imgmsg(img)  
        self.img_pub.publish(img_msg)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.init_scan_state = True

    def cmd_vel_raw_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_callback(self):
        if self.init_scan_state is True and self.object_detected is True:
            self.detect_obstacle()

    def odom_callback(self, msg):
        self.x_act = msg.pose.pose.position.x
        self.y_act = msg.pose.pose.position.y
        self.z_ori = msg.pose.pose.orientation.z
        # print ("Message received: ", self.x_act, self.y_act,self.z_ori)

    def movebase_client(self,goal_coordinates):
        # rclpy.init()

        # Create a ROS 2 node
        node = rclpy.create_node('movebase_client')

        # Create an action client called "navigate_to_pose" with action definition "NavigateToPose"
        client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

        # Wait until the action server has started up and started listening for goals
        if not client.wait_for_server(timeout_sec=5.0):
            print("Action server not available!")
            node.destroy_node()
            rclpy.shutdown()
            return

        # Define a list of goal coordinates
        # goal_coordinates = [
        #     (2.0, 0.0, 0.0),  # x, y, theta for Goal 1
        #     (1.0, -3.0, 0.0),  # x, y, theta for Goal 2
        #     (1.0, 0.1, 0.5),  # x, y, theta for Goal 3
        # ]
        # goal_coordinates = [
        #     (-1.0+2.0, -3.0+0.63, 0.0),  # x, y, theta for Goal 1
        # ]


        # Loop over the goal coordinates and send goals one by one
        while True:
            for goal_x, goal_y, goal_theta in goal_coordinates:
                # Create a goal message with the PoseStamped message
                goal_msg = NavigateToPose.Goal()

                # Set the goal target pose
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.pose.position.x = goal_x
                goal_msg.pose.pose.position.y = goal_y
                goal_msg.pose.pose.orientation.z = goal_theta

                # Send the goal to the action server
                goal_handle_future = client.send_goal_async(goal_msg)

                # Wait for the goal to complete
                while rclpy.ok():
                    rclpy.spin_once(node)
                    if goal_handle_future.done():
                        goal_handle = goal_handle_future.result()
                        if goal_handle.accepted:
                            print("Goal accepted.")
                            result_future = goal_handle.get_result_async()
                            rclpy.spin_until_future_complete(node, result_future)
                            result = result_future.result().result
                            if result!=False:
                                print("Goal execution done!")
                            else:
                                print("Goal execution failed!")
                            break
                        else:
                            print("Goal rejected.")
                            break

        # Clean up resources
        node.destroy_node()
        # rclpy.shutdown()

    def detect_obstacle(self):
        twist = Twist()

        # if self.left_bound<self.image_width//2 and self.right_bound>self.image_width//2:#just do forward
        #     obstacle_distance = self.scan_ranges[0]
        #     angle = 0
        # else:
        #     forward_pixel_length = self.image_width/2/math.tan(31.1/180*math.pi)#half the fov angle
        #     tolerance = 2#push closer to object
        #     if self.left_bound>=self.image_width//2:#object on right half of camera
        #         #forward is 0 degrees, then follow astc increment of angle where right is 270 degrees
        #         angle = 360-round(math.atan(abs(self.left_bound-self.image_width//2)/forward_pixel_length)/math.pi*180)#obtain left most point angle
        #         obstacle_distance = self.scan_ranges[angle-tolerance]
        #     elif self.right_bound<=self.image_width//2:#object on right half of camera
        #         angle = round(math.atan(abs(self.right_bound-self.image_width//2)/forward_pixel_length)/math.pi*180)#obtain left most point angle
        #         obstacle_distance = self.scan_ranges[angle+tolerance]

        centre_of_picture = self.image_width/2
        centre_of_object = (self.right_bound+self.left_bound)//2

        forward_pixel_length = centre_of_picture/math.tan(31.1/180*math.pi)#half the fov angle
        tolerance = 2#push closer to object
        if centre_of_object>=centre_of_picture:#centre on right side of image
            #forward is 0 degrees, then follow astc increment of angle where right is 270 degrees
            angle = 360-round(math.atan(abs(centre_of_object-centre_of_picture)/forward_pixel_length)/math.pi*180)#obtain left most point angle
        elif centre_of_object<centre_of_picture:#centre on left side of image
            angle = round(math.atan(abs(centre_of_object-centre_of_picture)/forward_pixel_length)/math.pi*180)#obtain left most point angle
        obstacle_distance = self.scan_ranges[angle]        


        safety_distance = 1  # unit: m
        self.get_logger().info(f"Potted plant detected at {obstacle_distance}m away at {angle}deg.")
        self.emergency_changed_goal = False
        if obstacle_distance > safety_distance:
            # twist.linear.x = self.linear_velocity
            # twist.angular.z = self.angular_velocity
            self.emergency_changed_goal = False
        else:
            if not self.emergency_changed_goal:
                self.emergency_changed_goal = True
                goal_coordinates = [[self.x_act+2.0,self.y_act+0.63,self.z_ori]]#give own cooridnates as new goal with adjustments

                self.movebase_client(goal_coordinates)                


            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)    



if __name__ == '__main__':
    rclpy.init(args=None)
    yolo_subscriber = Yolo_subscriber()
    camera_subscriber = Camera_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(yolo_subscriber)
    executor.add_node(camera_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    rate = yolo_subscriber.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
