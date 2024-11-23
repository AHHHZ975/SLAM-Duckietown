"This program undistorts images from duckietown."
import yaml
import numpy as np
import os
import cv2 as cv2
with open("luthor_intrinsic.yaml") as file:
    camera_list = yaml.load(file,Loader = yaml.FullLoader)

camera_intrinsic_matrix = np.array(camera_list['camera_matrix']['data']).reshape(3,3)

distortion_coeff = np.array(camera_list['distortion_coefficients']['data']).reshape(5,1)

images_data_dir = "/home/asha/SLAMDuck/luthor-2019-11-23-19-29-02/luthor_2019-11-23-19-29-02_images/"
undistorted_images_data_dir ="/home/asha/SLAMDuck/luthor-2019-11-23-19-29-02/luthor-2019-11-23-19-29-02-undistored/"
#get all the images
images_filenames = sorted(os.listdir(images_data_dir))
image_1 = cv2.imread(images_data_dir + images_filenames[0])
height = image_1.shape[0]
width = image_1.shape[1]


newmatrix, roi = cv2.getOptimalNewCameraMatrix(camera_intrinsic_matrix,distortion_coeff,(width,height),1, (width,height))
# new_image= cv2.undistort(img,camera_intrinsic_matrix,distortion_coeff,newmatrix)


map_x, map_y = cv2.initUndistortRectifyMap(camera_intrinsic_matrix, distortion_coeff, None, newmatrix, (width,height), cv2.CV_32FC1)



for i in images_filenames:

    print(images_data_dir + i)
    img = cv2.imread(images_data_dir +i)
    new_image = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)

    #print(undistorted_images_data_dir+ "undistorted_%s" %i)
    cv2.imwrite(undistorted_images_data_dir+ "undistorted_%s" %i, new_image)
