#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ranger_msgs.msg import ActuatorStateArray
from ranger_msgs.msg import ActuatorState

class JointToRangerPublisher(Node):
    def __init__(self):
        super().__init__('joint_to_ranger_publisher')
        self.publisher_ = self.create_publisher(ActuatorStateArray, '/ranger_states', 10)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.velocity = 0.0
        self.angle = 0.0

    def listener_callback(self, msg):
        ranger_msg = ActuatorStateArray()

        # Assuming msg.name, msg.position, msg.velocity, msg.effort have the same length
        for i in range(len(msg.name)):
            actuator_state = ActuatorState()
            actuator_state.id = i
            if len(msg.velocity) and len(msg.position) != 0:
                if i < 4:
                    actuator_state.motor.driver_state = msg.position[i+4]
                    self.velocity = msg.position[i+4]
                else:
                    actuator_state.motor.driver_state = msg.velocity[i-4]
                    self.angle = msg.velocity[i-4]
            else:
                if i < 4:
                    actuator_state.motor.driver_state = self.velocity
                else:
                    actuator_state.motor.driver_state = self.angle
            
            ranger_msg.states.append(actuator_state)

        self.publisher_.publish(ranger_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_to_ranger_publisher = JointToRangerPublisher()
    rclpy.spin(joint_to_ranger_publisher)
    # Destroy the node explicitly
    joint_to_ranger_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()