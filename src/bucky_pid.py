#!/usr/bin/env python
import bucky_env
import numpy as np
import simple_plot
import time

action = {"jump":0.0, "motor1":0.0, "motor2":0.0}

def compute_action(state):
    x_ref = 0.0
    y_ref = 0.0
    x_angle = state[3]
    y_angle = state[4]
    x_err = x_ref - x_angle
    y_err = y_ref - y_angle
    with open("k_value.txt", "r") as txt:
        x_KP = float(txt.readline())
    action["motor1"] = x_KP * x_err
    # action["motor1"] = 1.0\
    # action["motor1"] = (state[6]/1000.0 + x_KP*x_err)
    # action["motor1"] = 0.2

    # print x_KP
    # if np.abs(action["motor1"]) <  0.001:
    #   action["motor1"] = 0.0
    # action["motor2"] = y_KP*y_err

    # action["motor2"] = state[7]/30.0 + y_KP*y_err

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
    if (np.abs(x_err) > 0.2) or (np.abs(y_err) > 0.2):
        action["motor1"] = 0.0
        action["motor2"] = 0.0

    return action, x_err, y_err

def main():
    env = bucky_env.BuckyEnv()
    simple_plotter = simple_plot.SimplePlot(3)
    last_time = 0.0
    while(True):
        # print "delta T: ", (time.time()-last_time)
        last_time = time.time()
        success = env.update_state()
        if success:
            state = env.state
            action, x_err, y_err = compute_action(state)
            env.take_action(action)
            formatted_state = [ '%.2f' % elem for elem in state ]

            print "state, action, ref, k: ", formatted_state, action, [x_err, y_err]
            # data_to_plot = [x_err*2.5, action['motor1'], state[6]/500.0]
            data_to_plot = [x_err, action['motor1'], state[6]/500.0]
            # print "action: ", action
        else:
            data_to_plot = [0.0, 0.0, 0.0]
        simple_plotter.update(data_to_plot)


if __name__ == '__main__':
    main()