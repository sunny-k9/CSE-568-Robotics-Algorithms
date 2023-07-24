#!/usr/bin/env python
import math
import tf

import matplotlib.pyplot as plt
from matplotlib import animation
import rospy
import numpy as np
from std_msgs.msg import String

start_pos = (9, 9)
X_len = 20
Y_len = 20
X = np.zeros((X_len, 1), dtype=np.float)
Y = np.zeros((Y_len, 1), dtype=np.float)
X_all = []
Y_all = []
X[start_pos[0]] = 1
Y[start_pos[1]] = 1

state = ()

first_touch = False


# current_pos = start_pos

def motion_callback(data):
    global X, Y, X_len, Y_len
    msg = data.data

    # inp_str = input("Enter a message: ")
    dir_str = msg[0].upper()
    # steps = int(msg[1:]) # put inside action or observation

    if dir_str == "U":
        steps = int(msg[1:])
        for i in range(steps):
            Y_new = np.zeros((Y_len, 1), dtype=np.float)

            for j in range(Y_len):
                if Y[j] > 0.05:
                    if (Y_len - 1) - j > 2:

                        Y_new[j] += Y[j] * 0.2
                        Y_new[j + 1] += Y[j] * 0.6
                        Y_new[j + 2] += Y[j] * 0.2
                    else:  # wall case
                        if (Y_len - 1) - j == 1:
                            Y_new[j] += Y[j] * 0.2
                            Y_new[j + 1] += Y[j] * 0.8
                        elif (Y_len - 1) - j == 0:
                            Y_new[j] += Y[j] * 1

            # print(Y_new)
            Y = Y_new
            nu = 1 / sum(Y)
            Y *= nu

    if dir_str == "D":
        steps = int(msg[1:])
        for i in range(steps):
            Y_new = np.zeros((Y_len, 1), dtype=np.float)

            for j in range(Y_len):
                if Y[j] > 0.05:
                    if j >= 2:
                        Y_new[j] += Y[j] * 0.2
                        Y_new[j - 1] += Y[j] * 0.6
                        Y_new[j - 2] += Y[j] * 0.2
                    else:  # wall case
                        if j == 1:
                            Y_new[j] += Y[j] * 0.2
                            Y_new[j - 1] += Y[j] * 0.8
                        elif j == 0:
                            Y_new[j] += Y[j] * 1

            Y = Y_new
            nu = 1 / sum(Y)
            Y *= nu

    if dir_str == "L":
        steps = int(msg[1:])
        for i in range(steps):
            X_new = np.zeros((X_len, 1), dtype=np.float)
            for j in range(X_len):
                if X[j] > 0.05:
                    if j >= 2:
                        X_new[j] += X[j] * 0.2
                        X_new[j - 1] += X[j] * 0.6
                        X_new[j - 2] += X[j] * 0.2
                    else:  # wall case
                        if j == 1:
                            X_new[j] += X[j] * 0.2
                            X_new[j - 1] += X[j] * 0.8
                        elif j == 0:
                            X_new[j] += X[j] * 1
            X = X_new
            nu = 1 / sum(X)
            X *= nu

    if dir_str == "R":
        steps = int(msg[1:])
        for i in range(steps):
            X_new = np.zeros((X_len, 1), dtype=np.float)

            for j in range(X_len):
                if X[j] > 0.05:
                    if (X_len - 1) - j > 2:
                        X_new[j] += X[j] * 0.2
                        X_new[j + 1] += X[j] * 0.6
                        X_new[j + 2] += X[j] * 0.2
                    else:  # wall case
                        if (X_len - 1) - j == 1:
                            X_new[j] += X[j] * 0.2
                            X_new[j + 1] += X[j] * 0.8
                        elif (X_len - 1) - j == 0:
                            X_new[j] += X[j] * 1

            X = X_new
            nu = 1 / sum(X)
            X *= nu

    if dir_str == "X":
        pos = int(msg[1:])
        X_new = np.zeros((X_len, 1), dtype=np.float)

        X_new[pos] = X[pos] * 0.4
        if pos - 2 >= 0:
            X_new[pos - 2] = X[pos - 2] * 0.1
        if pos - 1 >= 0:
            X_new[pos - 1] = X[pos - 1] * 0.2
        if pos + 1 < X_len:
            X_new[pos + 1] = X[pos + 1] * 0.2
        if pos + 2 < X_len:
            X_new[pos + 2] = X[pos + 2] * 0.1

        X = X_new
        nu = 1 / sum(X)
        X *= nu

    if dir_str == "Y":
        pos = int(msg[1:])
        Y_new = np.zeros((Y_len, 1), dtype=np.float)

        Y_new[pos] = Y[pos] * 0.4
        if pos - 2 >= 0:
            Y_new[pos - 2] = Y[pos - 2] * 0.1
        if pos - 1 >= 0:
            Y_new[pos - 1] = Y[pos - 1] * 0.2
        if pos + 1 < X_len:
            Y_new[pos + 1] = Y[pos + 1] * 0.2
        if pos + 2 < X_len:
            Y_new[pos + 2] = Y[pos + 2] * 0.1

        Y = Y_new
        nu = 1 / sum(Y)
        Y *= nu

    print("X: ", X.T)
    print("Y: ", Y.T)
    print()


def update_map(frame):
    global img, Y, X
    prob_map = Y @ X.T
    #print(prob_map)
    img.set_array(prob_map)

    return


if __name__ == '__main__':

    try:
        rospy.init_node("lab9", anonymous=True)
        rospy.Subscriber("/robot", String, motion_callback)
        # update_plot(None)
        prob_map = Y @ X.T
        fig_map = plt.figure(figsize=(20, 20))
        img = plt.imshow(prob_map, origin="lower", cmap=plt.cm.get_cmap("viridis"))
        anim = animation.FuncAnimation(fig=fig_map, func=update_map)

        plt.colorbar()
        plt.show(block=True)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
