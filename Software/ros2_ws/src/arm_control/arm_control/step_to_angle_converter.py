import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState


class StepToAngleConverter(Node):
    def __init__(self):
        super().__init__('step_to_angle_converter')
        
        # System-Parameter
        self.declare_parameter('steps_per_rev', 3200)
        self.declare_parameter('gear_ratio', 20.0)
        #System parameter aus arm_parameter.yaml auslesen
        self.STEPS_PER_REV = self.get_parameter('steps_per_rev').value
        self.GEAR_RATIO = self.get_parameter('gear_ratio').value

        #jointnamen und startposition definieren(auf URDF anpassen) weil MOVEIT u RVIZ wollen standardisierte nachrichten
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.joint_positions = [0.0] * 6        #startposition=0position


        self.sub = self.create_subscription(Int32MultiArray, '/joint_steps', self.step_callback, 10)    #subscibe to steps
        self.pub = self.create_publisher(JointState, '/joint_states', 10)                  #publish standadised /jointsates for MOVIT u Rviz

        # Initialen Zustand publizieren (Nullstellung) siehe funktion 
        self.publish_joint_states() 

    def step_callback(self, msg):
        #schaut ob 3 Werte ankommen, msg.data ist das was reinkommt durch /joint_steps
        if len(msg.data) < 3:
            return

        axis = msg.data[0]  #welche achse
        steps = msg.data[1] #wie viele schritte
        direction = 1.0 if msg.data[2] == 1 else -1.0   #welche richtung
        #schaut ob die achse existeirt(im rahmen 0 bis 5-->6 achsen ist)
        if 0 <= axis < 6:
            # 1. Delta in Grad berechnen
            delta_deg = direction * (steps / self.STEPS_PER_REV) * (360.0 / self.GEAR_RATIO)   
            
            # 2. In Radiant umrechnen und Position aktualisieren
            self.joint_positions[axis] += math.radians(delta_deg)

            # 3. Aktualisierten Zustand senden wieder namen und position als standadisiertes topic
            self.publish_joint_states()

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