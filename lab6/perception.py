import rospy
import random
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math

points = []

degrees = np.linspace(-1 * np.pi / 2, np.pi / 2, 361)


def get_point_cloud(range_data):
    global degrees
    range_data[np.where(range_data == 3.0)] = 0  # wrong points given for no wall because max range = 3

    sin_degrees = np.sin(degrees)
    cos_degrees = np.cos(degrees)

    points_X = cos_degrees * range_data
    points_Y = sin_degrees * range_data

    temp_points_X = points_X[(points_Y != 0) & (points_X != 0)]
    points_Y = points_Y[(points_Y != 0) & (points_X != 0)]
    points_X = temp_points_X

    return points_X, points_Y


def ransac(points_X, points_Y, k, dist_thresh):
    points_len = len(points_X)
    inliers = []
    outliers = []
    max_inliers_count = 0
    best_line = []
    if points_len > 0:
        for _ in range(k):
            idx1 = np.random.randint(0, points_len - 1)
            idx2 = np.random.randint(0, points_len - 1)
            while idx1 == idx2:
                idx2 = np.random.randint(0, points_len - 1)
            if idx1 != idx2:
                inlier_count = 0
                outlier_count = 0

                p1 = (points_X[idx1], points_Y[idx1])  # if using norm for dist between line and point use np.array()

                p2 = (points_X[idx2], points_Y[idx2])  # if using norm for dist between line and point use np.array()

                for i in range(points_len):
                    compare_point = (
                    points_X[i], points_Y[i])  # if using norm for dist between line and point use np.array()

                    # print(compare_point)
                    # d = np.abs(np.linalg.norm(np.cross(p2 - p1, p1 - compare_point)) / np.linalg.norm(p2 - p1))
                    d = abs(((p2[0] - p1[0]) * (p1[1] - compare_point[1])) - (
                            (p1[0] - compare_point[0]) * (p2[1] - p1[1]))) / math.sqrt(
                        math.pow(p2[0] - p1[0], 2) + math.pow(p2[1] - p1[1], 2))

                    if d < dist_thresh:
                        inlier_count += 1
                    else:
                        outlier_count += 1
                    if inlier_count > max_inliers_count:
                        max_inliers_count = inlier_count
                        best_line = [p1, p2]
                # print(best_line)
    return best_line


def ransac_process(data):
    range_data = np.array(data.ranges)

    # rospy.Rate(10)

    bool = 0
    # print(range_data.shape)
    global pub
    if np.shape(range_data)[0] > 0:
        points_X, points_Y = get_point_cloud(range_data)
        best_line = ransac(points_X, points_Y, k=3, dist_thresh=0.5)
        marker_wall = Marker()
        marker_wall.header.frame_id = "base_laser_link"
        marker_wall.header.stamp = rospy.Time.now()
        marker_wall.lifetime = rospy.Duration(0)
        marker_wall.id = 0
        marker_wall.type = Marker.LINE_STRIP
        marker_wall.action = Marker.ADD
        marker_wall.scale.x = 0.05
        marker_wall.color.g = 1.0
        marker_wall.color.a = 1.0

        if len(best_line) > 0:
            p1 = Point()

            p1.x = best_line[0][0]
            p1.y = best_line[0][1]

            p2 = Point()
            p2.x = best_line[1][0]
            p2.y = best_line[1][1]

            marker_wall.points.append(p1)
            marker_wall.points.append(p2)

            pub.publish(marker_wall)


if __name__ == '__main__':
    try:
        rospy.init_node('ransac', anonymous=True)
        pub = rospy.Publisher("visualization_marker", Marker, queue_size=1)

        rospy.Subscriber("base_scan", LaserScan, ransac_process)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
