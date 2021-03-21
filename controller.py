import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from curtsies import Input



class Controller(Node):

	def __init__(self):  

		print("readya1a")
		super().__init__('Controller')
		print("ready1b")
		self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
		self.declare_parameter('up', 'w')
		self.declare_parameter('down', 's')
		self.declare_parameter('left', 'a')
		self.declare_parameter('right', 'd')
		print("ready1c")
		self.up = 'w'
		self.down = 's'
		self.left = 'a'
		self.right = 'd'
		timer_period = 0.1
		print("ready1d")
		self.timer = self.create_timer(timer_period, self.callback)
		print("ready1e")
		self.msg=Twist()
		print("ready1f")

		
	def custom_params(self):

		self.up = self.get_parameter('up').get_parameter_value().string_value
		self.down = self.get_parameter('down').get_parameter_value().string_value
		self.left = self.get_parameter('left').get_parameter_value().string_value
		self.right = self.get_parameter('right').get_parameter_value().string_value

	def callback(self):

		self.custom_params()
		with Input(keynames='curses') as input_generator:
			for key in input_generator:

				if key == self.up:
					self.msg.linear.x = 2.0
					self.msg.angular.z = 0.0

				if key == self.down:
					self.msg.linear.x = -2.0
					self.msg.angular.z = 0.0

				if key == self.right:
					self.msg.angular.z = 0.0
					self.msg.linear.x = 2.0

				if key == self.left:
					self.msg.angular.z = 0.0
					self.msg.linear.x = -2.0

		self.publisher_.publish(self.msg)

def main(args = None):

	print("ready1")
	rclpy.init(args=args)
	print("ready2")
	controller = Controller()
	print("ready3")
	rclpy.spin(controller)
	controller.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":

	main()
