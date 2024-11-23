import numpy as np
from scipy.interpolate import interp1d
import pylab as plt

# The address to the text files were generated after reading the recorded bag file
wheel_file_path = '/home/asha/SLAMDuck/data_dec_17/velocity_luthor_2019-12-17-23-32-14/luthor_2019-12-17-23-32-14.txt'
image_file_path = '/home/asha/SLAMDuck/data_dec_17/velocity_luthor_2019-12-17-23-32-14/luthor_2019-12-17-23-32-14_image_time.txt'

# An output path to store the velocity and the corresponding timesteps, I think
out_path = '/home/asha/SLAMDuck/data_dec_17/velocity_time_luthor_2019-12-17-23-32-14/luthor_2019-12-17-23-32-14_'

# Load the data from the text files. They are Numpy values
wheel_data = np.loadtxt(wheel_file_path)
image_time = np.loadtxt(image_file_path)

# Decoding the data into left and right wheels' velocities
l_vel = wheel_data[:,0]
r_vel = wheel_data[:,1]
wheel_time = wheel_data[:,2]

# Convert the timesteps to seconds
image_time = image_time/1e9
wheel_time = wheel_time/1e9

# Median time between images
median_image_time = np.median(np.diff(image_time))



# Find large gaps in the image_time and fill them with wheel time
gapless_image_time = []
has_image = []
thresh = 0.2
for i in range(len(image_time)-1):
    # Add the image time, which we know has an associated image
    gapless_image_time.append(image_time[i])
    has_image.append(True)

    # Check if there's a gap between this image timestamp and the
    # next image timestamp. If this is too big (>thresh), then we
    # will fill it with times from the wheel velocity data
    if (image_time[i+1] - image_time[i]) > thresh:
        wheel_time_to_add = np.arange(image_time[i], image_time[i+1], median_image_time)
        has_image += [False] * len(wheel_time_to_add)
        gapless_image_time += wheel_time_to_add.tolist()

gapless_image_time = np.array(gapless_image_time)
has_image = np.array(has_image)

plt.figure()
plt.plot(np.diff(image_time),label='image times')
plt.legend()
plt.figure()
plt.plot(np.diff(wheel_time),label='wheel times')
plt.legend()
plt.figure()
plt.plot(np.diff(gapless_image_time),label='gapless wheel times')
plt.title('Gaps in the image times have been filled in\nwith wheel times (when available)')
plt.legend()
plt.show()


# Interpolation functions
l_interp_func = interp1d(x=wheel_time,y=l_vel,kind='linear',bounds_error=False,fill_value=0)
r_interp_func = interp1d(x=wheel_time,y=r_vel,kind='linear',bounds_error=False,fill_value=0)

l_vel_interp = l_interp_func(gapless_image_time)
r_vel_interp = r_interp_func(gapless_image_time)

plt.figure()
plt.plot(wheel_time,l_vel,'.k',label='measured wheel velocity')
plt.plot(gapless_image_time[has_image],l_vel_interp[has_image],'xb',label='interpolated velocities from times with images')
plt.plot(gapless_image_time[np.logical_not(has_image)],l_vel_interp[np.logical_not(has_image)],'xr',label='interpolated velocities from times without images')
plt.title('left wheel')
plt.legend()

plt.figure()
plt.plot(wheel_time,r_vel,'.k',label='measured wheel velocity')
plt.plot(gapless_image_time[has_image],r_vel_interp[has_image],'xb',label='interpolated velocities from times with images')
plt.plot(gapless_image_time[np.logical_not(has_image)],r_vel_interp[np.logical_not(has_image)],'xr',label='interpolated velocities from times without images')
plt.title('right wheel')
plt.legend()
plt.show()

#write  out
np.savetxt(out_path+'interpolated_time.txt',np.transpose((has_image.astype(int),gapless_image_time)),
    header='HAS_IMAGE (0=false,1=true), TIME')
np.savetxt(out_path+'interpolated_wheel_velocities.txt',np.transpose((l_vel_interp,r_vel_interp)),
           header='Left velocity, right velocity')


#how to load these files:
# d = np.loadtxt(out_path+'interpolated_time.txt')