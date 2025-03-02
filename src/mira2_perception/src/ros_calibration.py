#!/usr/bin/env python3

import cv2
import numpy as np
import os
import shutil
import rospy 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

save_dir = 'images/'
save_dir_calibration_files = "~/"
if(not os.path.exists("images")):
    os.mkdir("images")
else:
    print("bruh")


bridge = CvBridge()

count = 0 
stop=False
def image_callback(msg):
    global count
    global stop
    if stop:
        return

    try:
        frame = bridge.imgmsg_to_cv2(msg,"bgr8")
    except Exception as e:
        print("Error ho gaya convert nahi ho raha",e)
        return
    
    frame = cv2.flip(frame,1)

    cv2.imshow("frame",frame)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('k'):
        cv2.imwrite('{}img_{}.jpg'.format(save_dir, count), frame)
        print("Saved image")
        count += 1
    elif(k == ord('q')):
        rospy.loginfo("Quitting the window")
    #    cv2.destroyAllWindows()
        stop = True

def main():
    global stop
    global count
    rospy.init_node("Camera_Calibrator",anonymous=True)
    sub = rospy.Subscriber("/camera_sony/image_raw",Image,image_callback)

    # rospy.spin()
   # while not stop:
    #    continue

    sub.unregister()
    
    print("boobie")
    chessboard_size = (7, 7) #(number_squares_x - 1, number_squares_y - 1)
    square_size = 0.017 #m
    image_path = 'images/'
    image_size = (640, 480) #(w,h)

    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp = objp * square_size

    object_points = []
    image_points = []

    images = os.listdir(image_path)

    cnt = 0
    print("Entering loop")
    for image in images:
        frame = cv2.imread('{}{}'.format(image_path, image))
        print(f"Processing image {cnt} of {len(images)}")
        cnt += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        print("stage 1 over")


        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        print("stage 2 over")


        if ret:
            object_points.append(objp)

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
            corners_final = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            print("stage 3 over")

            image_points.append(corners_final)

            img = cv2.drawChessboardCorners(frame, chessboard_size, corners_final, ret)

            # cv2.imshow("points", img)
            # cv2.waitKey(100)
        else:
            print("no chessboard corners found")
    print("FINISHED PROCESSING")
   # cv2.destroyAllWindows()

    shutil.rmtree("images")

    rep, camera_matrix, dist_coeff, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, image_size, None, None)

    np.savez("cam.npz", camera_matrix=camera_matrix, dist_coeff=dist_coeff, rvecs=rvecs, tvecs=tvecs)



if __name__ == "__main__":
    main()
