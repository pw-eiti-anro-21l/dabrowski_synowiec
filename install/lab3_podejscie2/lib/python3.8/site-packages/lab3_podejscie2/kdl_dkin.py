import os
import rclpy
import mathutils
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped
import json
from rclpy.clock import ROSClock
from PyKDL import *
import yaml

class Kdl_dkin(Node):

	def __init__(self):
		super().__init__('Kdl_dkin')
		self.subscription = self.create_subscription(JointState,'joint_states',self.listener_callback,10)
		self.subscription
		qos_profile = QoSProfile(depth=10)
		self.pose_publisher = self.create_publisher(PoseStamped, '/pose_stamped_kdl', qos_profile)

	def listener_callback(self, msg):
	
		values = readYAMLfile()
		chain = Chain()
		base_link1 = Joint(Joint.RotZ)
		frame1 = Frame(Rotation.RPY(values[0][0],values[0][1],values[0][2]),
			Vector(values[1][0],values[1][1],values[1][2])) 
		segment1 = Segment(base_link1,frame1)
		chain.addSegment(segment1) 


		link2_link1 = Joint(Joint.RotZ) 
		frame2 = Frame(Rotation.RPY(values[2][0],values[2][1],values[2][2]),
			Vector(values[3][0],values[3][1],values[3][2]))
		segment2=Segment(link2_link1,frame2)
		chain.addSegment(segment2)


		link2_link3 = Joint(Joint.TransZ) 
		frame3 = Frame(Rotation.RPY(values[4][0],values[4][1],values[4][2]),
			Vector(values[5][0],values[5][1],values[5][2]))
		segment3=Segment(link2_link3,frame3)
		chain.addSegment(segment3)



		joint_positions=JntArray(3)
		joint_positions[0]= msg.position[0]
		joint_positions[1]= msg.position[1]
		joint_positions[2]= -msg.position[2]



		fk=ChainFkSolverPos_recursive(chain)
		finalFrame=Frame()
		fk.JntToCart(joint_positions,finalFrame)

		qua = finalFrame.M.GetQuaternion()

		xyz = finalFrame.p

		poses = PoseStamped()
		now = self.get_clock().now()
		poses.header.stamp = ROSClock().now().to_msg()
		poses.header.frame_id = "base"

		poses.pose.position.x = xyz[0]
		poses.pose.position.y = xyz[1]
		poses.pose.position.z = xyz[2]
		poses.pose.orientation = Quaternion(w=float(qua[3]), x=float(qua[0]), y=float(qua[1]), z=float(qua[2]))
		self.pose_publisher.publish(poses)

def readYAMLfile():


	with open(os.path.join(
		get_package_share_directory('lab3_podejscie2'),'URDF.yaml'), 'r') as file:

		data = yaml.load(file, Loader=yaml.FullLoader)

	my_data=[]

	joint1_RPY = data['i1']['j_rpy']
	joint1_Vector = data['i1']['j_xyz']
	joint2_RPY = data['i2']['j_rpy']
	joint2_Vector = data['i2']['j_xyz']
	joint3_RPY = data['i3']['j_rpy']
	joint3_Vector = data['i3']['j_xyz']

	my_data.extend((joint1_RPY,joint1_Vector,joint2_RPY,joint2_Vector,joint3_RPY,joint3_Vector))

	values = []
	for element in my_data:
		new_element = element.split()
		list_of_floats = [float(item) for item in new_element]
		values.append(list_of_floats)


	return values



def main(args=None):
    rclpy.init(args=args)

    kdl = Kdl_dkin()
    rclpy.spin(kdl)

    kdl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
