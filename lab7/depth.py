import glob
import cv2
import struct
import rospy
import numpy as np
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header
from sensor_msgs import point_cloud2

l_image = None
r_image = None
frame_id = rospy.get_param("frame_id")
frame_count = 0
cam_info = None
K = [[684.517517, 0, 648.407166], [0, 648.517517, 343.224548], [0, 0, 1]]



def left_image(img_msg):
    global l_image, frame_count
    # cv2.namedWindow("left image", cv2.WINDOW_NORMAL)
    bridge = CvBridge()
    try:
        l_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
    except Exception as e:
        pass
        #print(e)

    frame_count += 1


def right_image(img_msg):
    global r_image
    # cv2.namedWindow("right image", cv2.WINDOW_NORMAL)
    bridge = CvBridge()
    try:
        r_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        publish_depth_image(bridge)
    except Exception as e:
        pass
        #print(e)


def left_caminfo(cinfo):
    global cam_info
    cam_info = cinfo


def publish_depth_image(bridge):
    global frame_id, frame_count, K
    # stereo = cv2.StereoSGBM_create(numDisparities=128, blockSize=21, speckleRange=16, minDisparity=4,
    #                                speckleWindowSize=45)

    min_disparities = -117
    max_disparities = -8
    window_size = 3
    stereo = cv2.StereoSGBM_create(numDisparities=max_disparities - min_disparities, minDisparity=min_disparities,
                                   blockSize=192,
                                   disp12MaxDiff=100,
                                   uniquenessRatio=150,
                                   speckleWindowSize=100,
                                   speckleRange=200)
    l_image_gray = cv2.cvtColor(l_image, cv2.COLOR_BGR2GRAY)
    r_image_gray = cv2.cvtColor(r_image, cv2.COLOR_BGR2GRAY)
    disparity = stereo.compute(l_image_gray, r_image_gray).astype(np.float32) / 16.0
    # norm_image = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    # disparity = norm_image

    # TODO: try centering cx,cy values currently zero 100 -w/2 0-10 h/2
    Q = np.array([[1, 0, 0, -K[0][2]], [0, 1, 0, -K[1][2]], [0, 0, 0, K[0][0]],
                  [0, 0, 1 / 0.12, 0]], dtype=np.float32)

    # TODO: change variable name, send depth img
    curr_frame_id = cam_info.header.seq
    print(curr_frame_id)
    if frame_id == curr_frame_id:
        points = cv2.reprojectImageTo3D(disparity, Q=Q)
        colors = cv2.cvtColor(l_image, cv2.COLOR_BGR2RGB)
        mask = disparity > disparity.min()
        out_points = points[mask]
        out_colors = colors[mask]

        # out_colors = out_colors.reshape(-1, 3)
        # rospy.loginfo(out_colors.shape)
        # frame_id = 100000
        # print(out_points.shape)
        # out_points[:, 2] = np.interp(out_points[:, 2], (out_points[:, 2].min(), out_points[:, 2].max()), (0.5, 25))

        points_ros = []

        for i in range(len(out_points)):
            x = out_points[i][0]
            z = out_points[i][1]
            y = out_points[i][2]

            # print(out_points[i][2])
            r = int(out_colors[i][0])
            g = int(out_colors[i][1])
            b = int(out_colors[i][2])
            a = 255

            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

            pt = [x, y, z, rgb]
            points_ros.append(pt)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1)
                  ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        pc2 = point_cloud2.create_cloud(header, fields, points_ros)

        pub_pcl2.publish(pc2)
        # rospy.sleep(0.5)

    disparity_bagged = bridge.cv2_to_imgmsg(disparity)
    pub.publish(disparity_bagged)


if __name__ == '__main__':
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('lab7')
    images = glob.glob(package_path + '/images//*.JPG')
    try:
        # TODO: Remove comment

        rospy.init_node('lab7', anonymous=True)
        rospy.Subscriber("zed/zed_node/left/image_rect_color", Image, left_image)
        rospy.Subscriber("zed/zed_node/right/image_rect_color", Image, right_image)
        rospy.Subscriber("zed/zed_node/left/camera_info", CameraInfo, left_caminfo)

        # rospy.Subscriber("zed/zed_node/")
        pub = rospy.Publisher("depth_image", Image, queue_size=1)
        pub_pcl2 = rospy.Publisher("point_cloud2", PointCloud2, queue_size=100)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
