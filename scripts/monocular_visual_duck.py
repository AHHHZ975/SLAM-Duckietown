import numpy as np
import cv2
from matplotlib import pyplot as plt

'''
img = cv2.imread('/home/asha/SLAMDuck/images/undistorted_frame000170.png',0)

# Initiate FAST object with default values
fast = cv2.FastFeatureDetector(threshold=3)

# find and draw the keypoints
kp = fast.detect(img,None)
img2 = cv2.drawKeypoints(img, kp, color=(255,0,0))

# Print all default params
print("Threshold: ", fast.getInt('threshold'))
print("nonmaxSuppression: ", fast.getBool('nonmaxSuppression'))
print("neighborhood: ", fast.getInt('type'))
print("Total Keypoints with nonmaxSuppression: ", len(kp))

cv2.imwrite('fast_true.png',img2)

# Disable nonmaxSuppression
#fast.setBool('nonmaxSuppression',0)
kp = fast.detect(img,None)

print("Total Keypoints without nonmaxSuppression: ", len(kp))

img3 = cv2.drawKeypoints(img, kp, color=(255,0,0))

cv2.imwrite('fast_false.png',img3)

'''
import numpy as np
import cv2
from matplotlib import pyplot as plt

img1 = cv2.imread('/home/asha/SLAMDuck/images/undistorted_frame000168.png')
gray1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)

corners1 = cv2.goodFeaturesToTrack(gray1,50,0.01,10)

img2 = cv2.imread('/home/asha/SLAMDuck/images/undistorted_frame000169.png')
gray2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)

#corners2 = cv2.goodFeaturesToTrack(gray2,50,0.01,10)

print(type(corners1))


#nextPts is the calculated new positions of input features in the second image
#each element of the vector is set to 1 if the flow for the corresponding features has been found, otherwise it is 0
#winsize: size of the search window
nextPts, status, error = cv2.calcOpticalFlowPyrLK(img1, img2, corners1, (21,21),2)
print(nextPts.shape)
print(nextPts[0])
#we must now get rid of points for which the KLT failed or are no longer in new

#print(status)
#print(flower_rider[0].shape)
#print(corners2)
#print(corners)

'''




#corners = np.int0(corners)


for i in corners1:
    x,y = i.ravel()
    cv2.circle(img1,(x,y),3,255,-1)

plt.imshow(img1)
plt.show()



for i in nextPts:
    x,y = i.ravel()
    cv2.circle(img2,(x,y),3,255,-1)

plt.imshow(img2)
plt.show()

for i in corners2:
    x, y = i.ravel()
    cv2.circle(img2, (x, y), 3, 255, -1)

plt.imshow(img2)
plt.show()


'''