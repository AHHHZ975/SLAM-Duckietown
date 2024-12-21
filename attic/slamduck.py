import numpy as np
import cv2
import matplotlib.pyplot as plt
import seaborn as sns

# Loading the left and right wheels velocities from the generated text file by the "my_node.py" file.
velocities = np.loadtxt("/home/asha/SLAMDuck/data_dec_02/velocity_time_luthor_2019-12-02-18-53-07/interpolated_wheel_velocities.txt")
time = np.loadtxt("/home/asha/SLAMDuck/data_dec_02/velocity_time_luthor_2019-12-02-18-53-07/interpolated_time.txt")
range_bearing_dir = "/home/asha/SLAMDuck/data_dec_02/range_bearing_luthor_2019-12-02-18-53-07/"

T = 0.034

# Note that we will call the the estimates from the prediction step as x_check and P_check
# whereas the corrected pose shall be called as x_hat and P_hat

# Initial conditions
# We have 8 landmarks, pose <m_x,m_y>
# State vector is 3 + 8*2, where 3 for the robot's pose and 8*2 for x and y coordinates of landmark multiplied by the number of landmarks which is 8

x_0_hat = np.zeros((3+8*2,1))
init_landmark_cov =[10**10]*8*2 # The initial cov for the landmarks is inf because we have no idea where they are
P_0_hat = np.diag([0,0,0]+init_landmark_cov)

# Wrap angles to the range of [-pi, pi]
def wraptopi(theta):
    if theta > np.pi :
        theta = theta  - 2*np.pi
    elif theta < -np.pi:
        theta = theta + 2*np.pi
    else:
        theta = theta

    return theta

# Sorting the eigenvalues of the covariance matrix
def eigsorted(cov):
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    return vals[order], vecs[:,order]



eigen_vals =[]

# I guess each ellipsoid (representing the area in which the apriltags exist)
# will be represented by three parameters.
ellipse_width =[]
ellipse_height =[]
ellipse_angle =[]

nstd = 3
ax = plt.subplot()
cov = P_0_hat[0:2, 0:2]
vals, vecs = eigsorted(cov)
angle = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
w, h = 2 * nstd * np.sqrt(vals)
eigen_vals.append(vals)
ellipse_width.append(w)
ellipse_height.append(h)
ellipse_angle.append(angle)

# prediction step:
sin = np.sin
cos = np.cos
v_var = 0.1
om_var = 0.1
r_var = 0.1
b_var = 0.1
w_k = np.diag([v_var,om_var])
n_k = np.diag([r_var,b_var])

l = 0.1 # The distance between two wheels on the robot

# We will store our values here
X_estimated  = [x_0_hat]
P_estimated = [P_0_hat]
size = 3+8*2
frame_count = 0
previously_observed_landmarks =[]
landmark_names = [31, 96, 79, 23, 86, 7, 25, 32]
d = 0.06
for i in range(0,time.shape[0]):
    #prediction step
    vel_left = velocities[i][1]
    vel_right = velocities[i][0]
    translational_velocity = (vel_left+vel_right)/2
    rotational_velocity = (1/l)*(vel_right-vel_left)

    theta_k_1 = wraptopi(X_estimated[-1][2])
    P_hat = P_estimated[-1]
    F_K_1 = np.diag([1]*size)
    F_K_1[0][2] = -T*sin(theta_k_1)*translational_velocity
    F_K_1[1][2] = T*cos(theta_k_1)*translational_velocity
    w_jacobian_k = np.zeros((size,2))
    w_jacobian_k[0][0] = T * cos(theta_k_1)
    w_jacobian_k[1][0] = T * sin(theta_k_1)
    w_jacobian_k[2][1] = T
    Q_prime_k = np.dot(w_jacobian_k, np.dot(w_k, w_jacobian_k.T))
    P_check_k = np.dot(F_K_1, np.dot(P_hat, F_K_1.T)) + Q_prime_k

    X_check_k = np.zeros((size,1))
    X_check_k[0] = X_estimated[-1][0] + T * cos(theta_k_1) * translational_velocity #x
    X_check_k[1] = X_estimated[-1][1] + T * sin(theta_k_1) * translational_velocity #y
    X_check_k[2] = wraptopi(X_estimated[-1][2] + T*rotational_velocity) #theta

    X_estimated.append(X_check_k)
    P_estimated.append(P_check_k)


    #correction step only occurs if there is an observation
    if int(time[i][0]) == 1: #there is an observation

        landmarks = np.loadtxt(range_bearing_dir + "range_bearing_frame" + format (frame_count,'06') +".txt")
        if landmarks.shape == (3,):
            landmarks = landmarks.reshape(1,3)
        for j in range(landmarks.shape[0]):
            if landmarks[j][0] in landmark_names:

                landmark_index = landmark_names.index(landmarks[j][0])


                if int(landmarks[j][0]) in previously_observed_landmarks:
                    #do the usual


                    beta = X_check_k[landmark_index] - X_check_k[0] - d * cos(X_check_k[2])
                    gamma = l[j][1] - y_check_k - d * sin(th_check_k)
                    alpha = np.sqrt(beta ** 2 + gamma ** 2)
                    range_landmark = alpha
                    bearing_landmark = wraptopi(np.arctan2(gamma, beta) - th_check_k)
                    measurement_range = r[i][j]
                    measurement_bearing = wraptopi(b[i][j])
                    '''

                else:
                    previously_observed_landmarks.append((int(landmarks[j][0])))

        frame_count = frame_count + 1

print(previously_observed_landmarks)

'''
        beta = l[j][0] - x_check_k - d * cos(th_check_k)
        gamma = l[j][1] - y_check_k - d * sin(th_check_k)
        alpha = np.sqrt(beta ** 2 + gamma ** 2)
        range_landmark = alpha
        bearing_landmark = wraptopi(np.arctan2(gamma, beta) - th_check_k)
        measurement_range = r[i][j]
        measurement_bearing = wraptopi(b[i][j])

        if len(G_k) == 0:  # no landmarks yet
            G_k = np.array([[-beta / alpha, -gamma / alpha,
                                (beta * d * sin(th_check_k) - gamma * d * cos(th_check_k)) / alpha], \
                            [gamma / alpha ** 2, -beta / alpha ** 2,
                                (-d * sin(th_check_k) * gamma - d * cos(th_check_k) * beta) / alpha ** 2 - 1]])
            g_k = np.array([[range_landmark], [bearing_landmark]])
            measurements = np.array([[measurement_range], [measurement_bearing]])




    else:
        X_estimated.append(X_check_k)
        P_estimated.append(P_check_k)





    #uncertainty ellipse
    cov = P_check_k[0:2, 0:2]
    vals, vecs = eigsorted(cov)
    angle = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
    w, h = 2 * nstd * np.sqrt(vals)
    eigen_vals.append(vals)
    ellipse_width.append(w)
    ellipse_height.append(h)
    ellipse_angle.append(angle)




#to plot the trajectory and the uncertainty
x = []
y = []
for j in range(len(X_estimated)):
    x.append(X_estimated[j][0])
    y.append(X_estimated[j][1])


plt.plot(x,y)
plt.show()

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse



for i in range(0,len(ellipse_height),50):
    nstd = 3
    fig = plt.figure()
    ax = fig.add_subplot()

    #cov = np.diag([cov_estimated_x[-1], cov_estimated_y[-1]])
    #vals, vecs = eigsorted(cov)
    #theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))
    #w, h = 2 * nstd * np.sqrt(vals)
    ell = Ellipse(xy=(x[i], y[i]),
                  width=ellipse_width[i], height=ellipse_height[i],
                  angle=ellipse_angle[i], color='red')
    ell.set_facecolor('none')
    ax.add_artist(ell)
    plt.plot(x[i],y[i], marker='.', color = 'b', fillstyle='full',alpha=0.3)
    plt.plot(x,y)
    plt.ylim((-2.5,2.5))
    plt.xlim((-3,1))
    plt.show()
    #plt.close('all')