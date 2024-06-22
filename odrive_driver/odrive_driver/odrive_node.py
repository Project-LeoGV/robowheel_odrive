import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

from geometry_msgs.msg import Twist,Quaternion
from nav_msgs.msg import Odometry

# from std_srvs.srv import Trigger
from std_msgs.msg import Header
import tf_transformations
from odrive.enums import *
from std_msgs.msg import Float32
from .odrive_command import ODriveController
import math

# import odrive
# from odrive.enums import *
class ODriveNode(Node):

    def __init__(self, odrv0):
        super().__init__('driver')
        self.odrv0 = odrv0

        self.declare_parameter('wheel_base', 0.48)  # Distance between the wheels
        self.declare_parameter('wheel_radius', 0.08255)  # Radius of the wheels

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # self.declare_parameter('connection.timeout', 15, ParameterDescriptor(
        #     type=ParameterType.PARAMETER_INTEGER, description='ODrive connection timeout in seconds'))
        # self.declare_parameter('battery.max_voltage', 4.2 * 6, ParameterDescriptor(
        #     type=ParameterType.PARAMETER_DOUBLE, description='Max battery voltage'))
        # self.declare_parameter('battery.min_voltage', 3.2 * 6, ParameterDescriptor(
        #     type=ParameterType.PARAMETER_DOUBLE, description='Min battery voltage'))
        # self.declare_parameter('battery.topic', 'barrery_percentage', ParameterDescriptor(
        #     type=ParameterType.PARAMETER_STRING, description='Battery percentage publisher topic'))
        # self.declare_parameter('joint_state.topic', 'joint_state', ParameterDescriptor(
        #     type=ParameterType.PARAMETER_STRING, description='Joint State publisher topic'))
        
        self.axis0_vel_pub = self.create_publisher(
            Float32, 'axis0_vel_pub', 50)
        self.axis1_vel_pub = self.create_publisher(
            Float32, 'axis1_vel_pub', 50)

        self.axis0_pos_pub = self.create_publisher(
            Float32, 'axis0_pos_pub', 50)
        self.axis1_pos_pub = self.create_publisher(
            Float32, 'axis1_pos_pub', 50)
        
        # Publishers
        self.odom_publisher = self.create_publisher(Odometry, 'odom/unfiltered', 10)
        # self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscribers
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # #  Timers for publishing odometry and joint states

        self.odom_timer = self.create_timer(0.1, self.publish_odometry)  # 10 Hz
        # self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
       
        # # Variables to store the state
        self.current_velocity = Twist()
        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # self.current_joint_state = JointState()
        # self.current_odometry = Odometry()
        # self.connect_odrive_service = self.create_service(
        #     Trigger,
        #     'connect_odrive',
        #     self.connect_odrive_callback
        # )

        # self.request_state_service = self.create_service(
        #     AxisState,
        #     'request_state',
        #     self.request_state_callback
        # )

        # self.battery_percentage_publisher_ = self.create_publisher(
        #     Float32,
        #     self.get_parameter(
        #         'battery.topic').get_parameter_value().string_value,
        #     1
        # )
        # self.battery_percentage_publisher_timer = self.create_timer(
        #     10,
        #     self.battery_percentage_publisher_callback
        # )
        # self.joint_state_publisher_ = self.create_publisher(
        #     JointState,
        #     self.get_parameter(
        #         'joint_state.topic').get_parameter_value().string_value,
        #     10
        # )
        # self.joint_state_publisher_timer = self.create_timer(
        #     0.1,
        #     self.joint_state_publisher_callback
        # )

        timer_period = 1 / 100
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # self.axis0_vel_sub = self.create_subscription(
        #     Float32, 'axis0_vel_sub', self.axis0_vel_callback, 50)
        # self.axis1_vel_sub = self.create_subscription(
        #     Float32, 'axis1_vel_sub', self.axis1_vel_callback, 50)

        # self.axis0_pos_sub = self.create_subscription(
        #     Float32, 'axis0_pos_sub', self.axis0_pos_callback, 50)
        # self.axis1_pos_sub = self.create_subscription(
        #     Float32, 'axis1_pos_sub', self.axis1_pos_callback, 50)
        
    # def is_driver_ready(self):
    #     if self.driver:
    #         try:
    #             if self.driver.user_config_loaded:
    #                 return True
    #             else:
    #                 self.get_logger().warn('ODrive user config not loaded')
    #                 return False
    #         except:
    #             self.get_logger().error('Unexpected error:', sys.exc_info()[0])
    #             return False
    #     else:
    #         self.get_logger().debug('ODrive not connected')
    #         return False

    # """
    # AXIS_STATE_UNDEFINED = 0
    # AXIS_STATE_IDLE = 1
    # AXIS_STATE_STARTUP_SEQUENCE = 2
    # AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
    # AXIS_STATE_MOTOR_CALIBRATION = 4
    # AXIS_STATE_SENSORLESS_CONTROL = 5
    # AXIS_STATE_ENCODER_INDEX_SEARCH = 6
    # AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
    # AXIS_STATE_CLOSED_LOOP_CONTROL = 8
    # AXIS_STATE_LOCKIN_SPIN = 9
    # AXIS_STATE_ENCODER_DIR_FIND = 10
    # """
    # def request_state_callback(self, request: AxisState.Request, response: AxisState.Response):
    #     if self.is_driver_ready():
    #         if request.axis == 0:
    #             self.driver.axis0.requested_state = request.state
    #             self.driver.axis0.watchdog_feed()
    #             response.success = True
    #             response.state = self.driver.axis0.current_state
    #             response.message = f'Success'
    #         elif request.axis == 1:
    #             self.driver.axis1.requested_state = request.state
    #             self.driver.axis1.watchdog_feed()
    #             response.success = True
    #             response.state = self.driver.axis0.current_state
    #             response.message = f'Success'
    #         else:
    #             response.success = False
    #             response.message = f'Axis not exist'
    #     else:
    #         response.success = False
    #         response.message = f'ODrive not ready'
        
    #     return response


    def timer_callback(self):
        msg = Float32()
        msg.data = self.odrv0.left_vel_estimate_tps()*2*math.pi
        self.axis0_vel_pub.publish(msg)

        msg.data = self.odrv0.right_vel_estimate_tps()*2*math.pi
        self.axis1_vel_pub.publish(msg)

        msg.data = self.odrv0.left_pos_t()*2*math.pi
        self.axis0_pos_pub.publish(msg)

        msg.data = self.odrv0.right_pos_t()*2*math.pi
        self.axis1_pos_pub.publish(msg)

    # def axis0_vel_callback(self, msg):
    #     print(f'axis0 vel: {msg}')
    #     self.odrv0.command_velocity(0, msg.data)

    # def axis1_vel_callback(self, msg):
    #     self.get_logger().info(f'axis1 vel: {msg}')
    #     self.odrv0.command_velocity(1, msg.data)

    # def axis0_pos_callback(self, msg):
    #     self.get_logger().info(f'axis0 pos: {msg}')
    #     self.odrv0.command_position(0, msg.data)

    # def axis1_pos_callback(self, msg):
    #     self.get_logger().info(f'axis1 pos: {msg}')
    #     self.odrv0.command_velocity(1, msg.data)

    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().info(f'Received velocity command: {msg}')
        self.current_velocity = msg
        self.get_logger().info(f'Received velocity command: {msg}')
        # Convert Twist message to left and right wheel speeds

        left_wheel_speed = (msg.linear.x - (self.wheel_base / 2.0) * msg.angular.z)/(2*3.14*self.wheel_radius)
        right_wheel_speed = (msg.linear.x + (self.wheel_base / 2.0) * msg.angular.z)/(2*3.14*self.wheel_radius)

        # Drive the wheels
        self.odrv0.drive_tps(left_wheel_speed, right_wheel_speed)


    def publish_odometry(self):
        self.current_time = self.get_clock().now()
        dt = (self.current_time - self.last_time).nanoseconds / 1e9  # Time difference in seconds
        self.last_time = self.current_time

        # Calculate the distance each wheel has traveled
        delta_l = self.odrv0.left_vel_estimate_radps() * self.wheel_radius * dt #rps*R*t
        delta_r = self.odrv0.right_vel_estimate_radps() * self.wheel_radius * dt

        # Calculate the change in orientation
        delta_theta = (delta_r - delta_l) / (self.wheel_base/2.0)

        # Calculate the distance traveled by the robot
        delta_s = (delta_r + delta_l) / 2.0

        # Update the robot's position and orientation
        # if delta_theta != 0:
        #     radius = delta_s / delta_theta
        #     self.x += radius * math.sin(delta_theta + self.theta) - radius * math.sin(self.theta)
        #     self.y += radius * math.cos(delta_theta + self.theta) - radius * math.cos(self.theta)
        #     self.theta += delta_theta
        # else:
        #     self.x += delta_s * math.cos(self.theta)
        #     self.y += delta_s * math.sin(self.theta)

        self.x += delta_s * math.cos(self.theta)
        self.y += delta_s * math.sin(self.theta)
        self.theta += delta_theta
  

        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(tf_transformations.quaternion_from_euler(0, 0, self.theta))

        # odom_msg.pose.covariance[0] = 0.001
        # odom_msg.pose.covariance[7] = 0.001
        # odom_msg.pose.covariance[35] = 0.001

        odom_msg.twist.twist.linear.x = delta_s / dt
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = delta_theta / dt

        # odom_msg.twist.covariance[0] = 0.0001
        # odom_msg.twist.covariance[7] = 0.0001
        # odom_msg.twist.covariance[35] = 0.0001

        self.odom_publisher.publish(odom_msg)






    # def publish_joint_states(self):
    #     # Create and publish JointState message
    #     joint_state_msg = JointState()
    #     joint_state_msg.header = Header()
    #     joint_state_msg.header.stamp = self.get_clock().now().to_msg()
    #     joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
    #     joint_state_msg.position = [self.odrv0.left_pos(), self.odrv0.right_pos()]

    #     self.joint_state_publisher.publish(joint_state_msg)

    # def connect_odrive_callback(self, request: Trigger.Request, response: Trigger.Response):
    #     try:
    #         self.get_logger().info('Connecting to ODrive')
    #         self.driver = odrive.find_any(
    #             timeout=self.get_parameter(
    #                 'connection.timeout'
    #             ).get_parameter_value().integer_value)
    #         self.get_logger().info('ODrive connected')
    #         response.success = True
    #         response.message = f'Connected to {self.driver.serial_number}'
    #         if not self.driver.user_config_loaded:
    #             self.get_logger().warn('ODrive user config not loaded')
    #         self.driver.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    #         self.driver.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    #     except TimeoutError:
    #         response.success = False
    #         response.message = 'Timeout'
    #     except:
    #         response.success = False
    #         response.message = 'Unexpected error:', sys.exc_info()[0]

    #     return response

    # def battery_percentage_publisher_callback(self):
    #     if self.is_driver_ready():
    #         msg = Float32()
    #         msg.data = (
    #             self.driver.vbus_voltage -
    #             self.get_parameter(
    #                 'battery.min_voltage').get_parameter_value().double_value
    #         ) / (
    #             self.get_parameter('battery.max_voltage').get_parameter_value().double_value -
    #             self.get_parameter('battery.min_voltage').get_parameter_value().double_value)
    #         self.battery_percentage_publisher_.publish(msg)
    #         if msg.data < 0.2:
    #             self.get_logger().warn(
    #                 f'ODrive battery percentage low: {msg.data:0.2f}')
    #     else:
    #         self.get_logger().debug('ODrive not ready')

    # def joint_state_publisher_callback(self):
    #     if self.is_driver_ready():
    #         msg = JointState()
    #         msg.header.stamp = self.get_clock().now().to_msg()
    #         msg.position = [self.driver.axis0.encoder.pos_estimate,
    #                         self.driver.axis1.encoder.pos_estimate]
    #         msg.velocity = [self.driver.axis0.encoder.vel_estimate,
    #                         self.driver.axis1.encoder.vel_estimate]
    #         self.joint_state_publisher_.publish(msg)
    #     else:
    #         self.get_logger().debug('ODrive not ready')



def main(args=None):
    rclpy.init(args=args)

    odrv0 = ODriveController()

    odrive_node = ODriveNode(odrv0)
    # odrv0.encoder_offset_calibration()
    odrv0.arm_velocity_control()

    rclpy.spin(odrive_node)

    odrive_node.destroy_node()
    rclpy.shutdown()
