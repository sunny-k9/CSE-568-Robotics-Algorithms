import numpy as np
import cv2
import glob
import cv2
import struct
import rospy
import numpy as np
import rospkg


def getCameraMatrix(images):
    points = []

    degrees = np.linspace(-1 * np.pi / 2, np.pi / 2, 361)

    # termination criteria

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((8 * 5, 3), np.float32)
    objp[:, :2] = np.mgrid[0:8, 0:5].T.reshape(-1, 2)
    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.
    img_shape = None

    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape

        # cv2.imshow("ab", img)
        # cv2.imshow("aa", gray)
        # cv2.waitKey(0)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (8, 5), None)
        # cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (8, 5), (-1, -1), criteria)
            imgpoints.append(corners2)
            # Draw and display the corners
            # cv2.drawChessboardCorners(img, (8, 5), corners2, ret)
            # cv2.imshow('img', img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

    if img_shape != None:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape[::-1], None, None)
        return mtx


if __name__ == '__main__':
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('lab7')
    images = glob.glob(package_path + '/images//*.JPG')
    try:
        # TODO: Remove comment
        mtx = getCameraMatrix(images)
        print(mtx)
        rospy.init_node('lab7', anonymous=True)


    except rospy.ROSInterruptException:
        pass
