import os
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import CanState, ConeArrayWithCovariance, CarState


class controlPublisher(Node):
  def __init__(self): 
    super().__init__('control_publisher')

    # Subscription to get the current state of the state machine
    self.state_sub = self.create_subscription(
      CanState, "/ros_can/state", self.stateCallback, 10)
    
    # publisher for drive controls
    self.publisher = self.create_publisher(AckermannDriveStamped, '/cmd', 10)
    self.timer = self.create_timer(0.01, self.publishCommand)

    # subscriber for cone locations
    self.cone_sub = self.create_subscription(
      ConeArrayWithCovariance, "/cones", self.coneCallback, 10
    )
    
    #mission control
    self.auto = False
    self.current_state = CanState()
    
    self.algorithm_states = {
      CanState.AMI_ACCELERATION: self.algorithm2,
      CanState.AMI_TRACK_DRIVE: self.track_drive,
      CanState.AMI_SKIDPAD: self.algorithm3
    }

    self.car_position = self.create_subscription(
      CarState, "/ground_truth/state", self.positionCallback, 10)
    
    self.blue_cones = []
    self.yellow_cones = []

  #dummy algorithms
  def track_drive(self):
    drive = AckermannDriveStamped()
    drive.header.stamp = self.get_clock().now().to_msg()

    self.get_logger().info(f"Blue cone position : {self.blue_cones[0]}")
    self.get_logger().info(f"Yellow cone position : {self.yellow_cones[0]}")


    drive.drive.acceleration = 0.0
    drive.drive.speed = 1.0

    drive.drive.steering_angle = 0.0
    drive.drive.steering_angle_velocity = 0.0

    return(drive)
  
  def algorithm2(self):
    drive = AckermannDriveStamped()
    drive.header.stamp = self.get_clock().now().to_msg()
    
    drive.drive.speed = 0.0
    drive.drive.steering_angle = 0.0

    if (len(self.blue_cones) != 0 and len(self.yellow_cones) != 0):
      blue_cone_x = self.blue_cones[len(self.blue_cones) - 1][0]
      yellow_cone_x = self.yellow_cones[len(self.yellow_cones) - 1][0]  
      blue_cone_y = self.blue_cones[len(self.blue_cones) - 1][1] if blue_cone_x > 3 else self.blue_cones[len(self.blue_cones) - 2][1]
      yellow_cone_y = self.yellow_cones[len(self.yellow_cones) - 1][1] if yellow_cone_x > 3 else self.yellow_cones[len(self.yellow_cones) - 2][1]

      self.get_logger().info(f"Blue cone position : {self.blue_cones[len(self.blue_cones) - 1]}")
      self.get_logger().info(f"Yellow cone position : {self.yellow_cones[len(self.yellow_cones) - 1]}")

      diff = blue_cone_y + yellow_cone_y

      drive.drive.acceleration = 0.1
      drive.drive.speed = 0.4

      drive.drive.steering_angle = 2.0 if diff > 0 else -0.2
      drive.drive.steering_angle_velocity = 0.0

    return(drive)
  
  def algorithm3(self):
    drive = AckermannDriveStamped()
    drive.header.stamp = self.get_clock().now().to_msg()

    drive.drive.acceleration = 0.1
    drive.drive.speed = 0.4

    drive.drive.steering_angle = -0.524
    drive.drive.steering_angle_velocity = 0.0

    return(drive)


  def publishCommand(self):
    if self.auto == False:
      return

    drive = AckermannDriveStamped()

    if self.current_state in self.algorithm_states:
        algorithm_function = self.algorithm_states[self.current_state]
        drive = algorithm_function()  # Call the function to get an AckermannDriveStamped message
        self.publisher.publish(drive)
        # self.get_logger().info(f"Published command from {algorithm_function.__name__}")
    # else:
        # self.get_logger().warn(f"No algorithm found for state {self.current_state}")

  def stateCallback(self, msg):
    self.current_state = msg.ami_state

    if (msg.ami_state == CanState.AMI_MANUAL or msg.ami_state == CanState.AMI_NOT_SELECTED) and self.auto == True:
      self.auto = False
    elif (msg.ami_state != CanState.AMI_MANUAL and msg.ami_state != CanState.AMI_NOT_SELECTED) and self.auto == False:
      self.auto = True
  
  def positionCallback(self, msg):
    self.position = msg.pose

  def coneCallback(self, msg):
    blue_cones = []
    yellow_cones = []

    for cone in msg.blue_cones:
      blue_cones.append((cone.point.x, cone.point.y))

    for cone in msg.yellow_cones:
      yellow_cones.append((cone.point.x, cone.point.y))

    self.blue_cones = blue_cones
    self.yellow_cones = yellow_cones
    
def main():
  rclpy.init()
  control_publisher = controlPublisher()
  rclpy.spin(control_publisher)
  control_publisher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()