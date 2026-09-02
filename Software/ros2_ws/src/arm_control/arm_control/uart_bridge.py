import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import serial
import math

class UartBridgeNode(Node):
    def __init__(self):
        super().__init__('uart_bridge_node')
        
        self.declare_parameter('steps_per_rev', 3200)
        self.declare_parameter('gear_ratio', 20.0)
        self.STEPS_PER_REV = self.get_parameter('steps_per_rev').value
        self.GEAR_RATIO = self.get_parameter('gear_ratio').value

        self.joint_names = [
            'shoulder_joint1', 'shoulder_joint2', 'elbow_joint',
            'wrist_turn_joint', 'wrist_tilt_joint', 'gripper_turn_joint'
        ]
        self.current_positions = [0.0] * 6
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        self.home_service = self.create_service(
            Trigger, 
            '/home_arm', 
            self.home_callback
        )

        # --- NEU: Timer publiziert die Position kontinuierlich (20 Hz) ---
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("UART Bridge + Homing-Service gestartet!")

    # --- NEU: Die Timer-Schleife ---
    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        self.joint_state_pub.publish(msg)

    def home_callback(self, request, response):
        self.get_logger().info("Homing-Flag wird an STM32 gesendet...")
        # self.ser.write(b"HOME\n")
        
        response_str = "OK"  

        if response_str == "OK":
            self.current_positions = [0.0] * 6
            # Timer übernimmt ab hier sofort das Senden der Nullen an RViz!
            
            response.success = True
            response.message = "Homing erfolgreich! Roboter ist auf Nullpunkt."
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"STM32 Homing Fehler: {response_str}"
            self.get_logger().error(response.message)
            
        return response

    def execute_callback(self, goal_handle):
        self.get_logger().info("Execute-Befehl empfangen!")
        trajectory = goal_handle.request.trajectory

        # Namen der Gelenke direkt aus der Trajektorie ziehen fixed das mit dem das der eine reale so random sachen macht und nicht das was er soll 
        incoming_joint_names = trajectory.joint_names
        last_point = trajectory.points[-1] 

        #  Array für 6 Achsen initialisieren
        sorted_target_positions = [0.0] * 6

        #Jeden erwarteten Namen mit MoveIts Liste abgleichen fixed das mit dem das der eine reale so random sachen macht und nicht das was er soll 
        for i, expected_name in enumerate(self.joint_names):
            if expected_name in incoming_joint_names:
                moveit_index = incoming_joint_names.index(expected_name)
                sorted_target_positions[i] = last_point.positions[moveit_index]
            else:
                self.get_logger().warn(f"Gelenk '{expected_name}' nicht in MoveIt-Trajektorie gefunden!")

        
        steps = [int(rad * (self.STEPS_PER_REV / (2 * math.pi)) * self.GEAR_RATIO) for rad in sorted_target_positions]
        uart_command = f"MOVE J1:{steps[0]} J2:{steps[1]} J3:{steps[2]} J4:{steps[3]} J5:{steps[4]} J6:{steps[5]}\n"
        
        self.get_logger().info(f"Sende an STM32: {uart_command.strip()}")
        # self.ser.write(uart_command.encode('utf-8'))
        
        response = "OK"

        if response == "OK" :
            self.get_logger().info("STM32 hat die Bewegung erfolgreich beendet!")
            
            # Neue Winkel eintragen (Timer sendet sie ab jetzt automatisch)
            self.current_positions = list(sorted_target_positions)
            goal_handle.succeed()
        else:
            self.get_logger().error(f"Fehler vom STM32: '{response}'")
            goal_handle.abort()

        return FollowJointTrajectory.Result()

def main(args=None):
    rclpy.init(args=args)
    node = UartBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()