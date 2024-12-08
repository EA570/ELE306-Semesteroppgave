import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class BaatController(Node):
    def __init__(self):
        super().__init__('baat_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_forward(self, duration=2.0, speed=1.0):
        twist = Twist()
        twist.linear.x = -speed  # Positiv verdi for å gå fremover
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        time.sleep(duration)
        self.stop()

    def move_backward(self, duration=2.0, speed=1.0):
        twist = Twist()
        twist.linear.x = speed  # Negativ verdi for å gå bakover
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        time.sleep(duration)
        self.stop()

    def turn_right(self, duration=2.0, angular_speed=1.0):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -angular_speed  # Negativ verdi for å svinge til høyre
        self.publisher.publish(twist)
        time.sleep(duration)
        self.stop()

    def turn_left(self, duration=2.0, angular_speed=1.0):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed  # Positiv verdi for å svinge til venstre
        self.publisher.publish(twist)
        time.sleep(duration)
        self.stop()

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = BaatController()

    print("Starter om 5 sekunder..")
    time.sleep(5)
    
    # Eksempel på bevegelsessekvens
    controller.move_forward(duration=6.0, speed=1.0)
    controller.turn_right(duration=3.0, angular_speed=1.0)
    controller.move_forward(duration=7.0, speed=1.0)
    controller.turn_left(duration=3.0, angular_speed=1.0)

    controller.stop()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()