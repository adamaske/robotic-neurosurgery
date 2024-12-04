
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import socket 
import struct
import rclpy.time
from sensor_msgs.msg import JointState 
from std_msgs.msg import Header
#from geometry_msgs.msg import TransformStamped, Transform
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_geometry_msgs.tf2_geometry_msgs import Pose
import moveit_commander
import sys

class robot_client(Node):
    def __init__(self):
        super().__init__("RobotControllerTCPClient")
        self.get_logger().info("robot_client: Welcome!")

        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)

        moveit_commander.roscpp_initalize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander("move_group")

        self.connect()
        self.run_client()

    def publish_joint_angles(self, angles):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0
        joint_goal[6] = 0
        
        self.group.go(joint_goal, wait=True)

        self.group.stop()

        current_joints = self.group.get_current_joint_values()
        return current_joints

        self.publish_setup()
        msg = JointState()
        msg.header = Header()
        msg.name = ["shoulder_pan_joint", 
                    "shoulder_lift_joint", 
                    "elbow_joint", 
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint"]
        msg.position = angles
        msg.velocity = []
        msg.effort = []
        self.joint_publisher.publish(msg=msg)
        print(f"robot_client : Published to 'joint_states' -> {angles}")


    def connect(self):
        self.server_host = "192.168.50.53" # host ip
        self.server_port = 65432 # host port
        print(f"robot_client : Client connecting to {self.server_host}:{self.server_port}")
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #ipv4, tcp
        self.client_socket.connect((self.server_host, self.server_port)) # wait for accepted connection
        print(f"robot_client : Client connected succesfully to {self.server_host}:{self.server_port}")
        self.connected = True

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

                self.publish_joint_angles(float_array)
                


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
