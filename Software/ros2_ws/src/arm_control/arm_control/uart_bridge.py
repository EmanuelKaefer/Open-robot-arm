import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState      # standadisiert um RViz zu updaten
from std_srvs.srv import Trigger            # standadisiert für den Homing-Service
import serial
import math

class UartBridgeNode(Node):
    def __init__(self):
        super().__init__('uart_bridge_node')
        
        # Serieller Port (Anpassen: z. B. /dev/ttyACM0 oder /dev/ttyUSB0)
        # self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=30)

        # System-Parameter
        self.declare_parameter('steps_per_rev', 3200)
        self.declare_parameter('gear_ratio', 20.0)
        self.STEPS_PER_REV = self.get_parameter('steps_per_rev').value
        self.GEAR_RATIO = self.get_parameter('gear_ratio').value

        # --- NEU: Status-Variablen und RViz-Publisher ---
        # WICHTIG: Passe die Namen exakt an deine URDF an!
        self.joint_names = [
            'shoulder_joint1', 'shoulder_joint2', 'elbow_joint',
            'wrist_turn_joint', 'wrist_tilt_joint', 'gripper_turn_joint'
        ]
        self.current_positions = [0.0] * 6
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # --- Schnittstellen (Server) ---
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        # NEU: Service-Server für das Homing
        self.home_service = self.create_service(
            Trigger, 
            '/home_arm', 
            self.home_callback
        )

        # RViz beim Start direkt auf 0.0 initialisieren
        self.publish_current_states()
        self.get_logger().info("UART Bridge + Homing-Service gestartet!")

    # --- NEU: Hilfsfunktion zum Updaten von RViz ---
    def publish_current_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        self.joint_state_pub.publish(msg)

    # --- NEU: Homing Logik ---
    def home_callback(self, request, response):
        self.get_logger().info("Homing-Flag wird an STM32 gesendet...")
        
        # Sende das Flag an den STM32
        # self.ser.write(b"HOME\n")
        
        self.get_logger().info("Warte auf Bestätigung vom STM32...")
        # response_str = self.ser.readline().decode('utf-8').strip()
        response_str = "OK"  # Zum Testen simuliert

        if response_str == "OK":
            # 1. Setze interne Winkel wieder auf 0
            self.current_positions = [0.0] * 6
            # 2. Update an RViz schicken (Roboter springt in RViz auf Home)
            self.publish_current_states()
            
            response.success = True
            response.message = "Homing erfolgreich! Roboter ist auf Nullpunkt."
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"STM32 Homing Fehler: {response_str}"
            self.get_logger().error(response.message)
            
        return response

    # --- Vorhandene Bewegungs-Logik (Erweitert um RViz Update) ---
    def execute_callback(self, goal_handle):
        self.get_logger().info("Execute-Befehl empfangen!")
        trajectory = goal_handle.request.trajectory

        last_point = trajectory.points[-1] 
        target_positions = last_point.positions 

        steps = [int(rad * (self.STEPS_PER_REV / (2 * math.pi))*self.GEAR_RATIO) for rad in target_positions]
        uart_command = f"MOVE J1:{steps[0]} J2:{steps[1]} J3:{steps[2]} J4:{steps[3]} J5:{steps[4]} J6:{steps[5]}\n"
        
        self.get_logger().info(f"Sende an STM32: {uart_command.strip()}")
        # self.ser.write(uart_command.encode('utf-8'))

        self.get_logger().info("Warte auf Rückmeldung vom STM32...")
        # response = self.ser.readline().decode('utf-8').strip()
        response = "OK"

        if response == "OK" :
            self.get_logger().info("STM32 hat die Bewegung erfolgreich beendet!")
            
            # NEU: Übernimm die neuen Winkel und zeige sie in RViz!
            self.current_positions = list(target_positions)
            self.publish_current_states()
            
            goal_handle.succeed()
        else:
            self.get_logger().error(f"Fehler oder Timeout vom STM32! Empfangen: '{response}'")
            goal_handle.abort()

        result = FollowJointTrajectory.Result()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = UartBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()