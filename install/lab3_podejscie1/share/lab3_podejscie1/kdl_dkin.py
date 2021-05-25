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

		self.subscription = self.create_subscription(
			JointState,
			'joint_states',
			self.listener_callback,
			10)
		self.subscription


	def listener_callback(self, msg):

		vals = readYAMLfile()

		chain = Chain()
		base_link1 = Joint(Joint.TransZ) 
		frame1 = Frame(Rotation.RPY(vals[0][0],vals[0][1],vals[0][2]),
			Vector(vals[1][0],vals[1][1],vals[1][2])) 
		segment1 = Segment(base_link1,frame1)
		chain.addSegment(segment1) 


		link2_link1 = Joint(Joint.TransY) 
		frame2 = Frame(Rotation.RPY(vals[2][0],vals[2][1],vals[2][2]),
			Vector(vals[3][0],vals[3][1],vals[3][2]))
		segment2=Segment(link2_link1,frame2)
		chain.addSegment(segment2)


		link2_link3 = Joint(Joint.TransY) 
		frame3 = Frame(Rotation.RPY(vals[4][0],vals[4][1],vals[4][2]),
			Vector(vals[5][0],vals[5][1],vals[5][2]))
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

		
		qos_profile = QoSProfile(depth=10)
		pose_publisher = self.create_publisher(PoseStamped, '/pose_stamped_kdl', qos_profile)


		poses = PoseStamped()
		now = self.get_clock().now()
		poses.header.stamp = ROSClock().now().to_msg()
		poses.header.frame_id = "base"

		poses.pose.position.x = xyz[0]
		poses.pose.position.y = xyz[1]
		poses.pose.position.z = xyz[2]
		poses.pose.orientation = Quaternion(w=float(qua[0]), x=float(qua[1]), y=float(qua[2]), z=float(qua[3]))
		pose_publisher.publish(poses)



def readYAMLfile():


	with open(os.path.join(
		get_package_share_directory('lab3_podejscie1'),'URDF.yaml'), 'r') as file:

		data = yaml.load(file, Loader=yaml.FullLoader)

	my_data=[]

	joint1_RPY = data['i1']['j_rpy']
	joint1_XYZ = data['i1']['j_xyz']
	joint2_RPY = data['i2']['j_rpy']
	joint2_XYZ = data['i2']['j_xyz']
	joint3_RPY = data['i3']['j_rpy']
	joint3_XYZ = data['i3']['j_xyz']

	my_data.extend((joint1_RPY,joint1_XYZ,joint2_RPY,joint2_XYZ,joint3_RPY,joint3_XYZ))
	vals = []
	
	for element in my_data:
		new_element = element.split()
		list_of_floats = [float(item) for item in new_element]
		vals.append(list_of_floats)


	return vals



def main(args=None):
	rclpy.init(args=args)

	kdl = Kdl_dkin()
	rclpy.spin(kdl)

	kdl.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()




  
