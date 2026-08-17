import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import threading

class StepPublisher(Node):
    def __init__(self):
        super().__init__('step_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, '/joint_steps', 10)
        self.pub_uart_stm = self.create_publisher(String, '/uart_stm', 10)
        # Thread für Terminal-Eingabe, damit rclpy.spin nicht blockiert
        threading.Thread(target=self.get_user_input, daemon=True).start()

    def get_user_input(self):
        while rclpy.ok():
            try:
                print("\n--- Neue Bewegung eingeben ---")
                axis = int(input("Achse (0-5): "))
                steps = int(input("Anzahl Schritte: "))
                direction = int(input("Richtung (1 = CW, 0 = CCW): "))

                msg = Int32MultiArray()
                msg.data = [axis, steps, direction] #wird gesendet, also in der empfängerdatei mit msg.data[i] arbeiten
                self.publisher_.publish(msg)
                self.get_logger().info(f"Gesendet auf /joint_steps -> Achse: {axis}, Schritte: {steps}, Dir: {direction}")

                # 2. UART-String formatieren (z. B. "MOVE:A0:S1600:D1\n")
                cmd_msg = String()
                cmd_msg.data = f"MOVE:A{axis}:S{steps}:D{direction}\n"  #wird gesendet, also in der empfängerdatei mit msg.data[i] arbeiten
                self.pub_uart_stm.publish(cmd_msg)
                self.get_logger().info(f"Gesendet auf /uart_stm: MOVE:A{axis}:S{steps}:D{direction}\n")

            except ValueError:
                self.get_logger().error("Bitte nur Ganzzahlen eingeben!")

def main(args=None):
    rclpy.init(args=args)
    node = StepPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()