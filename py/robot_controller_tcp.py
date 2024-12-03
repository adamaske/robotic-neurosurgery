import rclpy
from rclpy.node import Node
import socket 

class TCPClientNode(Node):
    def __init__(self):
        super().__init__("RobotControllerTCPClient")
        self.get_logger().info("Hello from ROS2")
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Timer")
        pass


#def run_client(): #listens to host port
#    try:
#        server_host = "192.168.50.53" # host ip
#        server_port = 65432 # host port
#        print(f"robot_controller_tcp : Client connecting to {server_host}:{server_port}")
#        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #ipv4, tcp
#        client_socket.connect((server_host, server_port)) # wait for accepted connection
#        print(f"robot_controller_tcp : Client connected succesfully to {server_host}:{server_port}")
#
#        connected = True
#        while connected:
#            byte_data = client_socket.recv(1024) #recieve from server
#
#            if not byte_data: # still connected 
#                connected = False
#                print("robot_controller_tcp : Connection terminated...")
#                break
#
#
#
#        client_socket.close()   
#        print("robot_controller_tcp : Closed client")     
#
#    except(KeyboardInterrupt, ExternalShutdownException):
#        client_socket.close()    
#        print("robot_controller_tcp : Closed client")
def main(args=None):
    rclpy.init(args=args)

    client_node = TCPClientNode()
    rclpy.spin(client_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
