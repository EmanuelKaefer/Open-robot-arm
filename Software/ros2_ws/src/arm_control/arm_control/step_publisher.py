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
                steps= []

                for i in range(6):
                    var = int(input(f"Anzahl Schritte für achse:{i}"))
                    steps.append(var)

                msg = Int32MultiArray()
                msg.data = steps #wird gesendet, also in der empfängerdatei mit msg.data[i] arbeiten
                self.publisher_.publish(msg)
                self.get_logger().info(f"Gesendet auf /joint_steps -> Schritte: {steps}")

                # 2. UART-String formatieren (z. B. "MOVE:A0:S1600:D1\n")
                cmd_msg = String()
                cmd_msg.data = f"MOVE:S{steps}\n"  #wird gesendet, also in der empfängerdatei mit cmd_msg.data[i] arbeiten
                self.pub_uart_stm.publish(cmd_msg)
                self.get_logger().info(f"Gesendet auf /uart_stm: MOVE:S{steps}\n")

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