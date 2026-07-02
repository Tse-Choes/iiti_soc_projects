#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given drone.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/pid_error				/throttle_pid
								/pitch_pid
								/roll_pid
					
Rather than using different variables, use list. eg : self.desired_state = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_desired_state = 1, self.y_desired_state = 2
'''

# Importing the required libraries


from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from controller_msg.msg import PIDTune
from error_msg.msg import Error
import rclpy
from rclpy.node import Node


class Swift_Pico(Node):
	def __init__(self):
		super().__init__('pico_controller')  # initializing ros node with name pico_controller

		# This corresponds to your current position of drone. This value must be updated in your whycon callback
		self.current_pos = [0.0, 0.0, 0.0]


		# This corresponds to the setpoint you want the drone to reach or hold
		self.desired_state = [ -8, 3, 25]  # whycon marker at the position of the drone given in the scene. Make the whycon marker associated with position_to_hold drone renderable and make changes accordingly


		# Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = SwiftMsgs()
		self.cmd.rc_roll = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_throttle = 1500

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		
		self.declare_parameter('pitch_pid.Kp', 0)
		self.declare_parameter('pitch_pid.Ki', 0)
		self.declare_parameter('pitch_pid.Kd', 0)

		self.declare_parameter('roll_pid.Kp', 0)
		self.declare_parameter('roll_pid.Ki', 0)
		self.declare_parameter('roll_pid.Kd', 0)

		self.declare_parameter('throttle_pid.Kp', 0)
		self.declare_parameter('throttle_pid.Ki', 0)
		self.declare_parameter('throttle_pid.Kd', 0)

		self.Kp = []
		self.Ki = []
		self.Kd = []

		self.Kp.append(self.get_parameter('roll_pid.Kp').get_parameter_value().integer_value * 0.03)
		self.Kp.append(self.get_parameter('pitch_pid.Kp').get_parameter_value().integer_value * 0.03)
		self.Kp.append(self.get_parameter('throttle_pid.Kp').get_parameter_value().integer_value * 0.03)

		self.Ki.append(self.get_parameter('roll_pid.Ki').get_parameter_value().integer_value * 0.008)
		self.Ki.append(self.get_parameter('pitch_pid.Ki').get_parameter_value().integer_value * 0.008)
		self.Ki.append(self.get_parameter('throttle_pid.Ki').get_parameter_value().integer_value * 0.008)

		self.Kd.append(self.get_parameter('roll_pid.Kd').get_parameter_value().integer_value * 0.6)
		self.Kd.append(self.get_parameter('pitch_pid.Kd').get_parameter_value().integer_value * 0.6)
		self.Kd.append(self.get_parameter('throttle_pid.Kd').get_parameter_value().integer_value * 0.6)

		self.get_logger().info(f"Kp: {self.Kp}, Ki: {self.Ki}, Kd: {self.Kd}")

		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.prev_states= []
		self.processed_pos = [0.0, 0.0, 0.0]
		self.prev_pos = list(self.processed_pos)
		self.vel = [0.0, 0.0, 0.0]

		self.min_value = [1000, 1000, 1000]
		self.max_value = [2000, 2000, 2000]

		self.pos_error = Error()
		self.prev_error = Error()
		self.error_sum = [0.0, 0.0, 0.0]
		self.prev_error.roll_error = 0.0
		self.prev_error.pitch_error = 0.0
		self.prev_error.throttle_error = 0.0

		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit.
	
		self.sample_time = 0.01  # in seconds

		# Publishing /drone_command, /pid_error
		self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
		self.pos_error_pub = self.create_publisher(Error, '/pos_error', 10)
	
		# Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
		self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
		self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
		self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
		self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)

		self.arm()  # ARMING THE DRONE

		# Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)
		#self.timer = self.create_timer(self.sample_time, self.pid)


	def disarm(self):
		self.cmd.rc_roll = 1000
		self.cmd.rc_yaw = 1000
		self.cmd.rc_pitch = 1000
		self.cmd.rc_throttle = 1000
		self.cmd.rc_aux4 = 1000
		self.command_pub.publish(self.cmd)
		

	def arm(self):
		self.disarm()
		self.cmd.rc_roll = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_throttle = 1500
		self.cmd.rc_aux4 = 2000
		self.command_pub.publish(self.cmd)  # Publishing /drone_command


	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self, msg):
		self.current_pos[0] = msg.poses[0].position.x 
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.current_pos[1] = msg.poses[0].position.y
		self.current_pos[2] = msg.poses[0].position.z

		#average filter -- take average of last 3 pos
		self.prev_states.append(list(self.current_pos))
		if (len(self.prev_states)>3):
			self.prev_states.pop(0) 
		
		elif (len(self.prev_states) <= 1):
			self.prev_pos = list(self.current_pos) #initialising pre_pos at start
			self.get_logger().info("running")
		
		sum = [0.0, 0.0, 0.0]
		counter = 0
		for pose in self.prev_states:
			for i in [0, 1, 2]:
				sum[i] += pose[i]
			counter += 1
		for i in [0, 1, 2]:
			self.processed_pos[i] = sum[i]/counter
			
			self.vel[i] = self.processed_pos[i] - self.prev_pos[i]
		
		#self.get_logger().info(str(counter) + " prev: " + str(self.prev_pos) + "  curr: " + str(self.processed_pos) + "  vel: " + str(self.vel))
		#self.get_logger().info(str(self.prev_states))
		self.prev_pos = list(self.processed_pos)

		self.pid() #running pid
		
		
		
		

	
		#---------------------------------------------------------------------------------------------------------------


	# Callback function for /throttle_pid
	# This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
	def altitude_set_pid(self, alt):
		self.Kp[2] = alt.kp * 0.03  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.ki * 0.008
		self.Kd[2] = alt.kd * 0.6

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def pitch_set_pid(self, alt):
		self.Kp[0] = alt.kp * 0.03  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = alt.ki * 0.008
		self.Kd[0] = alt.kd * 0.6

	def roll_set_pid(self, alt):
		self.Kp[1] = alt.kp * 0.03  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = alt.ki * 0.008
		self.Kd[1] = alt.kd * 0.6
	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
		#-----------------------------Write the PID algorithm here--------------------------------------------------------------

		# Steps:
		# 	1. Compute error in each axis. eg: error[0] = self.current_pos[0] - self.desired_state[0] ,where error[0] corresponds to error in x...
		#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
		#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
		#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
		#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
		#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
		#																														self.cmd.rcPitch = self.max_values[1]
		#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
		#	8. Add error_sum
		fields = ['pitch_error', 'roll_error', 'throttle_error']		#(x, y, z) = (pitch, roll, throtle)
		i = 0
		p_term = [0.0, 0.0, 0.0]
		i_term = [0.0, 0.0, 0.0]
		d_term = [0.0, 0.0, 0.0]
		for field in fields:
			setattr(self.pos_error, field, self.processed_pos[i] - self.desired_state[i])
			self.error_sum[i] += getattr(self.pos_error, field)
			self.error_sum[i] = max(min(self.error_sum[i], 1000.0),-1000.0)

			p_term[i] = self.Kp[i]*getattr(self.pos_error, field)
			i_term[i] = self.error_sum[i]*self.Ki[i]
			d_term[i] = self.Kd[i]*self.vel[i]
			i += 1
			
		
		
		self.cmd.rc_pitch = 1500 - int(p_term[0] + i_term[0] + d_term[0])
		self.cmd.rc_roll = 1500 - int(p_term[1] + i_term[1] + d_term[1])
		self.cmd.rc_throttle = 1532 + int(p_term[2] + i_term[2] + d_term[2])
		
		self.prev_error = self.pos_error


	#------------------------------------------------------------------------------------------------------------------------
		self.command_pub.publish(self.cmd)
		# calculate throttle error, pitch error and roll error, then publish it accordingly
		self.pos_error_pub.publish(self.pos_error)



def main(args=None):
	rclpy.init(args=args)
	swift_pico = Swift_Pico()
 
	try:
		rclpy.spin(swift_pico)
	except KeyboardInterrupt:
		swift_pico.get_logger().info('KeyboardInterrupt, shutting down.\n')
	finally:
		swift_pico.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


if __name__ == '__main__':
	main()