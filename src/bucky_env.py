import serial
import math
import time
import numpy as np

bucky_serial = serial.Serial('/dev/ttyACM0',115200)
time.sleep(2)

def map_float(x, bounds):
	in_min, in_max, out_min, out_max = bounds
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def Quaternion_toEulerianAngle(x, y, z, w):
	ysqr = y*y
	
	t0 = +2.0 * (w * x + y*z)
	t1 = +1.0 - 2.0 * (x*x + ysqr)
	X = math.atan2(t0, t1)
	
	t2 = +2.0 * (w*y - z*x)
	t2 =  1 if t2 > 1 else t2
	t2 = -1 if t2 < -1 else t2
	Y = math.asin(t2)
	
	t3 = +2.0 * (w * z + x*y)
	t4 = +1.0 - 2.0 * (ysqr + z*z)
	Z = math.atan2(t3, t4)
	
	return X, Y, Z 

class BuckyEnv():
	# state:  accleration_x, accleration_y, accleration_z, angle_x, angle_y, angle_z,  motor_1_speed, motor_2_speed
	def __init__(self):
		self.actions = {"jump":0.0, "motor1":0.0, "motor2":0.0}
		self.action_dimensions = 3
		self.action_bounds = {"jump": [0.0,1.0,0.0,250.0], "motor1":[-1.0,1.0,-10.0,10.0], "motor2":[-1.0,1.0,-10.0,10.0]}
		self.state = [0.0]*8
		self.state_dimensions = 8
		self._data_flag = "999"
		self._action_names = ["jump", "motor1", "motor2"]
		self.x_ref = [0.0]*500
		self.x_avg = [0.0]*10
		bucky_serial.write("start")


	def update_state(self):
		state_string = bucky_serial.readline()
		state_string = state_string.strip().split(",")
		if state_string[0] == "111":
			raw_state = list(map(float,state_string))[1:]
			x_angle, y_angle, z_angle = Quaternion_toEulerianAngle(raw_state[3],raw_state[4],raw_state[5],raw_state[6])
			self.x_avg.pop(0)
			self.x_avg.append(x_angle)
			self.x_ref.pop(0)
			self.x_ref.append(x_angle)
			self.state[:3] = raw_state[:3]
			offset = sum(self.x_ref)/len(self.x_ref)
			self.state[3] = sum(self.x_avg)/len(self.x_avg) - offset
			# self.state[3] = x_angle 
			# offset = sum(self.x_ref)/len(self.x_ref)
			# self.state[3] = x_angle - offset
			# if np.abs(offset) > 0.05:
			# 	self.state[3] -= offset
			self.state[4] = y_angle
			self.state[5] = z_angle
			self.state[6:] = raw_state[7:]
			self.state[6] *= 1.0
			success = True
		else:
			print state_string
			success = False

		return success

	def take_action(self, actions):
		if (set(actions.keys()) == set(self._action_names)) and (len(actions.keys()) == len(self._action_names)):
			success = True
			self.actions = actions.copy()
			bounded_actions = actions.copy()
			for action_name in self._action_names:
				bounded_actions[action_name] = map_float(self.actions[action_name],self.action_bounds[action_name])
			action_string = self._data_flag
			action_string += ","
			action_string += str(bounded_actions["jump"])
			action_string += ","
			action_string += str(bounded_actions["motor1"])
			action_string += ","
			action_string += str(bounded_actions["motor2"])
			action_string += ","
			bucky_serial.write(action_string)
		else:
			success = False
			print "ERROR: Incorrect action format!"

		return success


