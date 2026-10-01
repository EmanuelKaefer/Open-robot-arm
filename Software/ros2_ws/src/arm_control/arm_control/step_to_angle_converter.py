import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class StepToAngleConverter(Node):
    def __init__(self):
        super().__init__('step_to_angle_converter')

        #jointnamen und startposition definieren(auf URDF anpassen) weil MOVEIT u RVIZ wollen standardisierte nachrichten
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.joint_positions = [0.0] * 6        #startposition=0position


        self.sub = self.create_subscription(Int32MultiArray, '/joint_steps', self.step_callback, 10)    #subscibe to steps
        self.pub = self.create_publisher(JointState, '/joint_states', 10)                  #publish standadised /jointsates for MOVIT u Rviz

        self.srv = self.create_service(Trigger, '/home_arm', self.home_callback)    #server für homing erstellen damit der arm wirklich auf 6*0 steht

        # Initialen Zustand publizieren (Nullstellung), wichtig damit MOVEIT2 startet müssen sofort werte auf Joint_states sein 
        self.publish_joint_states() 


    def home_callback(self, request, response):
        self.get_logger().info('Homing-Befehl empfangen. Kalibriere Nullpunkt...')

        # Hier würde später das Warten auf das Signal deiner Serial-Bridge / MCU stehen:
        # ------------------------------------------------------------------------
        # MCU fährt auf Endschalter -> meldet "OK" zurück...
        # ------------------------------------------------------------------------

        # Winkel im Code auf echten Nullpunkt zurücksetzen
        self.joint_positions = [0.0] * 6

        # Neue Nullstellung sofort auf joint_states senden
        self.publish_joint_states()

        # Antwort an den Service-Aufrufer schicken
        response.success = True
        response.message = 'Homing erfolgreich! Alle Winkel auf 0.0 rad gesetzt.'
        return response

    def publish_joint_states(self):
        msg = JointState()  #sagt das auf Jointstate gepublished wird
        msg.header.stamp = self.get_clock().now().to_msg()#schikt einen Zeitstempel mit um bewegungen zu timen
        msg.name = self.joint_names #schikt die namen
        msg.position = self.joint_positions #schikt die positionen

        #published die oben zusammengebaute nachricht auf Jointstate und gibt die nachricht aus
        self.pub.publish(msg)
        self.get_logger().info(f"JointStates publiziert: {list(round(p, 3) for p in self.joint_positions)}")

def main(args=None):
    rclpy.init(args=args)
    node = StepToAngleConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()