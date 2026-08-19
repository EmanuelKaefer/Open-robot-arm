import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
import serial
import math

class UartBridgeNode(Node):
    def __init__(self):
        super().__init__('uart_bridge_node')
        
        # Serieller Port (Anpassen: z. B. /dev/ttyACM0 oder /dev/ttyUSB0)
        # self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # System-Parameter
        self.declare_parameter('steps_per_rev', 3200)
        self.declare_parameter('gear_ratio', 20.0)
        #System parameter aus arm_parameter.yaml auslesen
        self.STEPS_PER_REV = self.get_parameter('steps_per_rev').value
        self.GEAR_RATIO = self.get_parameter('gear_ratio').value

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,  #standadisierter server an den MOVEIT die Gelenkwinkel sendet
            'arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        self.get_logger().info("UART Bridge Action Server gestartet!")

        #wird autmatisch aufgerufen sobald man in MOVEIT execute drückt
    def execute_callback(self, goal_handle):
        self.get_logger().info("Execute-Befehl empfangen!")
        trajectory = goal_handle.request.trajectory

        # Letzten Punkt der Flugbahn holen (Endwinkel)
        last_point = trajectory.points[-1]  #greift auf den letzten eintrag der liste mit positionen zu den MOVEIT sendet
        target_positions = last_point.positions # Werte in Radiant

        # Umrechnung Radiant -> Motor-Steps
        steps = [int(rad * (self.STEPS_PER_REV / (2 * math.pi))*self.GEAR_RATIO) for rad in target_positions]

        # UART String bauen: z. B. "MOVE J1:1000 J2:-500 ..."
        uart_command = f"MOVE J1:{steps[0]} J2:{steps[1]} J3:{steps[2]} J4:{steps[3]} J5:{steps[4]} J6:{steps[5]}\n"
        
        self.get_logger().info(f"Sende an STM32: {uart_command.strip()}")
        
        # An Seriellen Port senden
        # self.ser.write(uart_command.encode('utf-8'))

        self.get_logger().info("Warte auf Rückmeldung vom STM32...")
        #response = self.ser.readline().decode('utf-8').strip()
        response="OK"

        # 3. Auswertung der Antwort
        if response == "OK" :
            self.get_logger().info("STM32 hat die Bewegung erfolgreich beendet!")
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