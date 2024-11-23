import yaml
import numpy as np
import cv2 as cv2

# Reading the camera intrinsic parameters from a .yaml file
with open("luthor_intrinsic.yaml") as file:
    camera_list = yaml.load(file,Loader = yaml.FullLoader)

camera_intrinsic_matrix = np.array(camera_list['camera_matrix']['data']).reshape(3,3)

# It assumes that we have the distortion coefficient of the camera lenz. But, what if we don't have it in our case?
distortion_coeff = np.array(camera_list['distortion_coefficients']['data']).reshape(5,1)

# Loading only an image frame
img = cv2.imread('/home/asha/SLAMDuck/images/frame000170.png')
height = img.shape[0]
width = img.shape[1]


newmatrix, roi = cv2.getOptimalNewCameraMatrix(camera_intrinsic_matrix,distortion_coeff,(width,height),1, (width,height))
#new_image= cv2.undistort(img,camera_intrinsic_matrix,distortion_coeff,newmatrix)


map_x, map_y = cv2.initUndistortRectifyMap(camera_intrinsic_matrix, distortion_coeff, None, newmatrix, (width,height), cv2.CV_32FC1)

new_image = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)

cv2.imwrite('/home/asha/SLAMDuck/images/undistort_frame000170.png', new_image)

