
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import socket 
import struct

class robot_client(Node):
    def __init__(self):
        super().__init__("RobotControllerTCPClient")
        self.get_logger().info("robot_client: Welcome!")

        self.server_host = "192.168.50.53" # host ip
        self.server_port = 65432 # host port
        print(f"robot_client : Client connecting to {self.server_host}:{self.server_port}")
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #ipv4, tcp
        self.client_socket.connect((self.server_host, self.server_port)) # wait for accepted connection
        print(f"robot_client : Client connected succesfully to {self.server_host}:{self.server_port}")
        self.connected = True
        self.run_client()

    def run_client(self): #listens to host port
        try:
            
            while self.connected:
                byte_data = self.client_socket.recv(1024) #recieve from server

                if not byte_data: # still connected 
                    self.connected = False
                    print("robot_client : Connection terminated...")
                    break
                print(f"robot_client : Received {byte_data}")
                float_array = struct.unpack("<6f" , byte_data)
                thetas = float_array
                for i in range(len(thetas)):
                    print(f"Theta {i} : {thetas[i]}")

                    
                print(f"Received floats: {float_array}")

            self.client_socket.close()   
            print("robot_client : Closed client")     

        except(KeyboardInterrupt, ExternalShutdownException):
            self.client_socket.close()    
            print("robot_client : Closed client")

def main(args=None):
    rclpy.init(args=args)

    client = robot_client()
    rclpy.spin(client)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
