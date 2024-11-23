import matplotlib.pyplot as plt
import numpy as np

import sys
sys.setrecursionlimit(100000000)
#with open("velocity_data_velocity_data_luthor_2019-11-23-19-28-09.txt") as f:
with open("/home/asha/SLAMDuck/luthor-2019-11-23-19-29-02/data_luthor_2019-11-23-19-29-02/wheel_time_data_luthor_2019-11-23-19-29-02-test.txt") as f:
    content = f.readlines()
vel_left =[]
vel_right =[]
time = []
#x_new = []
#y_new =[]
for line in content:
    line_split = line.split(" ")
    left = float(line_split[0])
    vel_left.append(left)
    right = float(line_split[1])
    vel_right.append(right)
    time.append(float(line_split[2])/10**9)


time_diff = np.diff(time)
#plt.plot(time_diff)
#plt.ylim(0,0.2)
#plt.show()


#do odometry
#duckiebot is a differential drive
#assumptions, wheels have the same diameter of 2R
#the wheels are equidistant from the center of the bot, i.e l/2, where l is the length between the wheels

#kinematics assumptions
#rolling without sliding
#no sidways sliding

sum_time =[]

def wraptopi(theta):
    if theta > np.pi:
        theta = theta - 2*np.pi
    elif theta < theta - 2*np.pi:
        theta = theta + 2*np.pi
    else:
        theta = theta
    return theta

def differential_drive(x_prev, y_prev, theta_prev,dt,vel_left,vel_right):
    l =0.1
    translational_velocity = (vel_left+vel_right)/2
    rotational_velocity = (1/l)*(vel_right-vel_left)

    x_k = x_prev + dt*np.cos(theta_prev)*translational_velocity
    y_k = y_prev + dt*np.sin(theta_prev)*translational_velocity
    theta_k = theta_prev + dt*rotational_velocity
    theta_k = wraptopi(theta_k)

    return x_k, y_k, theta_k



x_new = [0]
y_new = [0]
theta_new = [0]

#lets drive the robot
for i in range(len(vel_left)-1):
    #throw in past state and current controls
    x_k, y_k, theta_k = differential_drive(x_new[-1],y_new[-1], theta_new[-1],time_diff[i],vel_left[i],vel_right[i])
    x_new.append(x_k)
    y_new.append(y_k)
    theta_new.append(theta_k)

    sum_time.append(time_diff[i])

plt.plot(x_new,y_new)
plt.show()