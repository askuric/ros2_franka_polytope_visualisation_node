import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
import pinocchio as pin
from rcl_interfaces.srv import GetParameters
from pycapacity.robot import *
import time
from visualization_msgs.msg import Marker
from . import polytope_visualisation_utils as pvu

# ROS2 Node class
class VelPolytopeNode(Node):
    def __init__(self):
        super().__init__('vel_polytope')
        
        # initial joint positions
        self.joint_positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        
        self.cli = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        if not self.cli.wait_for_service(timeout_sec=60.0):
            self.get_logger().error(f'Service not available: /robot_state_publisher/get_parameters')
            rclpy.shutdown(); return
        req = GetParameters.Request()
        req.names = ['robot_description']
        future = self.cli.call_async(req)
        future.add_done_callback(self._on_robot_description)
        
        self.marker_pub = self.create_publisher(Marker, '/polytope_marker', 1)
        
        # joint state subscriber
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.callback,
            10)
        
        # timer for main loop (15 Hz)
        self.timer = self.create_timer(1.0/15.0, self.timer_callback)
        
        self.get_logger().info('Velocity polytope node initialized')
    
    def _on_robot_description(self, future):
        response = future.result()
        robot_description = response.values[0].string_value
        # load robot model from URDF
        model = pin.buildModelFromXML(robot_description)
        self.robot = pin.RobotWrapper(model)
        self.get_logger().info('Robot model loaded from URDF')
        
        # end-effector frame id
        self.ee_frame_id = self.robot.model.getFrameId("fr3_link8")
        
    # function receiving the new joint positions
    def callback(self, data):
        self.joint_positions = np.array(data.position)
    
    # main loop callback
    def timer_callback(self):
        if not hasattr(self, 'robot'):
            return
        if np.sum(self.joint_positions):
            self.plot_polytope(self.robot, self.joint_positions, frame_name="fr3_link8", scaling_factor=10)


    def plot_polytope(self, robot, q, frame_name = None, scaling_factor = 10):

        # if no joint state received, return
        if not np.sum(q):
            return

        # if frame not specified, use the last frame
        if frame_name is None:
            frame_name = robot.model.frames[-1].name

        # calculate forward kinematics of the robot
        robot.forwardKinematics(q)
        ee_position = robot.data.oMf[robot.model.getFrameId(frame_name)].translation

        # calculate jacobi matrix
        robot.computeJointJacobians(q)
        J = pin.getFrameJacobian(robot.model, robot.data, robot.model.getFrameId(frame_name) , pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        # only position part
        J = J[:3,:]

        # maximal joint angles
        dq_max = robot.model.velocityLimit.T
        dq_min = -dq_max

        # calculate force vertexes
        start = time.time()
        poly = velocity_polytope(J, dq_max, dq_min)
        poly.find_faces()
        velocity_vertex, velocity_faces = poly.vertices/scaling_factor, poly.face_indices
        
        # Orient faces so normals point outward
        velocity_faces = pvu.orient_faces_outward(velocity_vertex, velocity_faces)
        
        # Publish polytope as marker
        faces_msg = pvu.create_polytope_triangles_msg(velocity_vertex, velocity_faces, ee_position, self.get_clock().now().to_msg())
        self.marker_pub.publish(faces_msg)
        # comment out for faster plotting
        edges_msg = pvu.create_polytope_edges_msg(velocity_vertex, velocity_faces, ee_position, self.get_clock().now().to_msg())
        self.marker_pub.publish(edges_msg)
        
    
# main function
def main(args=None):
    rclpy.init(args=args)
    node = VelPolytopeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()