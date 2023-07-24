import math
import tf

import matplotlib.pyplot as plt
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# TODO: add rosparam, remove comment(harcode)
goal_pos_x = rospy.get_param("goalx")
goal_pos_y = rospy.get_param("goaly")

goal = (goal_pos_x, goal_pos_y)
start_pos_x = -8.0
start_pos_y = -2.0
world_map = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0,  # 1
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0,  # 2
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  # 3
             1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  # 4
             0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  # 5
             0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,  # 6
             0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,  # 7
             0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,  # 8
             0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,  # 9
             0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,  # 10
             0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,  # 11
             0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,  # 12
             0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,  # 13
             0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  # 14
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  # 15
             0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,  # 16
             0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0,  # 17
             0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0,  # 18
             0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0,  # 19
             0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1]  # 20

world_map = np.array(world_map).reshape(20, 18)

orien = 0
pos = 0


class Node:
    def __init__(self, parent_node, position):
        self.position = position
        self.parent_node = parent_node

        self.g = 0
        self.h = 0
        self.f = 0


def calc_h(curr, gl):
    h = math.pow(curr[0] - gl[0], 2) + math.pow(curr[1] - gl[1], 2)
    return h


world_map = np.array(world_map).reshape((20, 18))

opened = []
closed = []

wrow, wcol = world_map.shape
start = Node(None, (start_pos_x, start_pos_y))
start.g = 0
start.h = 0  # calc_h(current.position, goal)
start.f = 0  # current.g + current.h
opened.append(start)

step_count = 0


def astar_single_pass(current_node):
    global opened, closed, goal, world_map, wrow, wcol
    goal = (math.ceil(goal[0]), math.ceil(goal[1]))

    top = Node(current_node, (current_node.position[0], current_node.position[1] + 1))
    left = Node(current_node, (current_node.position[0] - 1, current_node.position[1]))
    bottom = Node(current_node, (current_node.position[0], current_node.position[1] - 1))
    right = Node(current_node, (current_node.position[0] + 1, current_node.position[1]))

    # TODO: Boundary check
    if right.position[0] >= wcol / 2:
        right = None
    if top.position[1] >= wrow / 2:
        top = None
    if bottom.position[1] <= -(wrow / 2):
        bottom = None
    if left.position[0] <= -(wcol / 2):
        left = None

    neighbours = [top, left, bottom, right]

    for neighbour in neighbours:
        if neighbour is not None:
            trans_y = 10 - int(neighbour.position[1])
            trans_x = int(neighbour.position[0]) + 9

            if len([True for node in closed if node.position == neighbour.position]) == 0:
                if world_map[trans_y - 1][trans_x - 1] != 1 or ((trans_y == wrow) and (trans_x == wcol)):
                    g = current_node.g + 100
                    h = calc_h(neighbour.position, goal)
                    f = g + h
                    neighbour.g = g
                    neighbour.h = h
                    neighbour.f = f
                    if len([True for node in opened if
                            node.position == neighbour.position]) == 0:
                        opened.append(neighbour)


def a_star():
    global opened, closed, goal, world_map, goal_pos_x, goal_pos_y
    path = []
    while len(opened) > 0:

        current_node = min(opened, key=lambda x: x.f)
        if type(current_node) != Node:
            print("Gotcha")

        current_index = opened.index(current_node)
        opened.pop(current_index)
        closed.append(current_node)

        if calc_h(current_node.position, (goal_pos_x, goal_pos_y)) <= 0.5:

            current = current_node
            while current is not None:
                path.append((current.position[0] - 0.5, current.position[1] - 0.5))
                current = current.parent_node
            break

        astar_single_pass(current_node)
    path = path[::-1]
    for step in path:
        world_map[10 - int(step[1] + 0.5) - 1][9 + int(step[0] + 0.5) - 1] = 2
    print("")
    for row in world_map:
        line = []
        for col in row:
            if col == 1:
                line.append("1")
            elif col == 0:
                line.append(" ")
            elif col == 2:
                line.append("*")
        print(" ".join(line))

    return path


move_forward = False


def move_bot(data):
    global orien
    global pos
    global final_path
    global pub
    global move_forward
    pos = data.pose.pose.position
    orien = data.pose.pose.orientation
    heading = tf.transformations.euler_from_quaternion([
        orien.x,
        orien.y,
        orien.z,
        orien.w])[2]

    twist = Twist()
    if len(final_path) > 0:
        sub_goal = final_path[0]

        # move to a point(sub-goal)
        goal_yaw = math.atan2(sub_goal[1] - pos.y, sub_goal[0] - pos.x)

        align_angle = goal_yaw - heading
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        if align_angle != 0:
            twist.angular.z = align_angle
        pub.publish(twist)

        if abs(align_angle) < 0.3:
            move_forward = True

        if move_forward == True:
            twist.linear.x = 0.8
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            pub.publish(twist)

        if calc_h((pos.x, pos.y), (sub_goal[0], sub_goal[1])) < 0.1:
            move_forward = False
            final_path.pop(0)


final_path = None
if __name__ == '__main__':

    try:
        rospy.init_node("lab8", anonymous=True)
        final_path = a_star()

        pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/odom", Odometry, move_bot)
        plt.imshow(world_map)
        plt.show()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
