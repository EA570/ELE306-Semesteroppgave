import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')
        self.publisher = self.create_publisher(JointTrajectory, '/boat_arm_controller/joint_trajectory', 10)
        self.get_logger().info("RobotArmController initialized")

    def move_to_position(self, positions, duration=2.0):
        """
        Move the arm to the specified joint positions over the given duration.
        :param positions: A list of target joint positions for each joint.
        :param duration: Time in seconds for the arm to reach the target positions.
        """
        self.get_logger().info(f"Moving arm to position: {positions}")
        
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['arm_base_joint', 'link_1_armbaat_joint', 'link_2_armbaat_joint', 'link_3_armbaat_joint']
        
        point = JointTrajectoryPoint()
        point.positions = positions  # Target joint positions
        point.time_from_start.sec = int(duration)  # Duration to reach target position

        trajectory_msg.points.append(point)

        # Publish the trajectory message
        self.publisher.publish(trajectory_msg)

        # Wait for the arm to reach the target position
        time.sleep(duration)
        
        # Stop the arm after reaching the target position
        

    def stop(self):
        """
        Stop the robot arm by sending the current position as the target.
        """
        self.get_logger().info("Stopping arm")
        
        # To stop the arm, set target to current positions without moving.
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['arm_base_joint', 'link_1_armbaat_joint', 'link_2_armbaat_joint', 'link_3_armbaat_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0]  # Keep the current position
        point.time_from_start.sec = 1  # Short duration for stop command

        trajectory_msg.points.append(point)

        # Publish stop command
        self.publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotArmController()
    
    # Forsinkelse på 5 sekunder før første bevegelse
    time.sleep(5)
    
    # Move to initial position
    controller.move_to_position([0.0, 0.0, 0.0, 0.0], duration=2.0)

    # Move to a new position - ut til siden og nedover
    controller.move_to_position([-1.5, -1.2, -1.0, -0.8], duration=3.0)  # Justerte vinkler
    
    # Wait and move to another position - litt lenger ut og ned
    time.sleep(1)
    controller.move_to_position([-1.5, -0.5, -0.2, -0.2], duration=2.0)  # Justerte vinkler
    
    time.sleep(1)
    controller.move_to_position([-3.14, -0.5, -0.2, -0.2], duration=2.0)

    time.sleep(1)
    controller.move_to_position([-3.14, -1.5, -1.0, -0.8], duration=2.0)  # Justerte vinkler
    
    # Stop the arm
    controller.stop()
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()