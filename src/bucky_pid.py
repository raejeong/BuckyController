#!/usr/bin/env python
import bucky_env
import numpy as np

action = {"jump":0.0, "motor1":0.0, "motor2":0.0}

def compute_action(state):
	x_ref = 0.12
	y_ref = -0.10
	x_KP = 5.0
	y_KP = 5.0
	x_angle = state[4]
	y_angle = state[3]
	x_err = x_ref - x_angle
	y_err = y_ref - y_angle
	# action["motor1"] = x_err
	action["motor1"] = state[6]/30.0 + x_KP*x_err
	# if abs(x_err) > 0.05:
		# action["motor1"] = state[6]/30.0 + x_KP*x_err + 0.2
		# action["motor1"] = x_KP*x_err
	# action["motor2"] += y_KP*y_err
	if action["motor1"] > 1.0:
		action["motor1"] = 1.0
	if action["motor1"] < -1.0:
		action["motor1"] = -1.0
	if action["motor2"] > 1.0:
		action["motor2"] = 1.0
	if action["motor2"] < -1.0:
		action["motor2"] = -1.0
	if (np.abs(x_err) > 0.26) or (np.abs(y_err) > 0.26):
		action["motor1"] = 0.0
		action["motor2"] = 0.0

	return action, x_err, y_err

def main():
   env = bucky_env.BuckyEnv()
   while(True):
   	success = env.update_state()
   	if success:
   		state = env.state
   		action, x_err, y_err = compute_action(state)
   		env.take_action(action)
   		formatted_state = [ '%.2f' % elem for elem in state ]
   		print "state, action: ", formatted_state, action, [x_err, y_err]
   		# print "action: ", action

if __name__ == '__main__':
    main()