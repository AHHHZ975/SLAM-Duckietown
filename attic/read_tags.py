import json, codecs
from glob import glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import seaborn as sns


tag_detections_dir = "/home/asha/SLAMDuck/data_dec_02/tag_detections_luthor_2019-12-02-18-53-07/"
image_dir = "/home/asha/SLAMDuck/data_dec_02/undistorted_images_luthor_2019-12-02-18-53-07/"

cos = np.cos
sin = np.sin
theta = (10/360) * 2*np.pi
rotation = np.array([[1,0,0],[0, cos(theta), -sin(theta)],[0, sin(theta),cos(theta)]])

#get all json files
histogram=[]
all_json = sorted(glob(tag_detections_dir+'*.json'))

print(all_json)
x_landmark =[]
y_landmark = []
x_r_landmark =[]
y_r_landmark =[]
ranges_to_plot = list(range(500,700,5))
for file_path in all_json:
	file_text = codecs.open(file_path, 'r', encoding='utf-8').read() #see https://stackoverflow.com/questions/26646362/numpy-array-is-not-json-serializable
	file_data = json.loads(file_text)
	histogram.append(len(file_data))
	#print(file_path.split('/')[-1]+' has '+str(len(file_data))+' tags')

	file_id = file_path[-11:-5]
	image_path = image_dir+'undistorted_frame'+file_id+'.png'

	file_index = int(file_id)

	if len(file_data)>0:
		#Convert all of the lists back to numpy arrays
		x_landmark = []
		y_landmark = []
		x_r_landmark = []
		y_r_landmark = []
		tag_ids = []
		for i in range(len(file_data)):
			tag_dict = file_data[i]
			for k in tag_dict:
				if isinstance(tag_dict[k],list):
					tag_dict[k] = np.asarray(tag_dict[k])
			file_data[i] = tag_dict
			print(file_data[i]['pose_t'])
			pose = np.dot(rotation, file_data[i]['pose_t'])
			#x_r_landmark.append(pose[2])
			#y_r_landmark.append(-pose[0])
			rangesy = np.sqrt(pose[2]**2 + -pose[0]**2)
			#print(range)
			bearing = np.arctan2((-pose[0]),pose[2])
			#print(bearing)

			if file_index in ranges_to_plot:
			#print(i)
			#if file_path == "/home/asha/SLAMDuck/data_dec_02/tag_detections_luthor_2019-12-02-18-53-07/tag_detections_undistorted_frame000064.json":
				pose = np.dot(rotation,file_data[i]['pose_t'])

				x_landmark.append(file_data[i]['pose_t'][2])
				y_landmark.append(-file_data[i]['pose_t'][0])
				x_r_landmark.append(pose[2])
				y_r_landmark.append(-pose[0])
				tag_ids.append(file_data[i]['tag_id'])
		if len(x_landmark) > 0:
			print(file_path)

			fig_plot = plt.figure(figsize=(10,5))
			ax_plot = fig_plot.add_subplot(121)
			ax_imag = fig_plot.add_subplot(122)

			#plot image
			img = mpimg.imread(image_path)
			ax_imag.imshow(img)
			ax_imag.set_title(image_path.split('/')[-1])

			for ii in range(len(tag_ids)):
				ax_plot.plot(x_landmark[ii], y_landmark[ii], 'ob')
				ax_plot.plot(x_r_landmark[ii], y_r_landmark[ii], 'ok')
				ax_plot.text(x_r_landmark[ii], y_r_landmark[ii], '   '+str(tag_ids[ii]) )
				# plt.plot(x_landmark[-2],y_landmark[-2], 'ob')
			ax_plot.plot(0, 0, 'or')
			ax_plot.set_title(file_path.split('/')[-1]+'\n'+'# tags = '+str(len(x_landmark)))
			ax_plot.set_xlim((-0.05,3))
			ax_plot.set_ylim((-3,3))
			plt.show()
		#print('Tags loaded and data converted to numpy format')

		#file_data[0]['pose_t']))
		#file_data[1]['pose_t']
		#file_data is a list of dictionaries. Each dictionary represents one of the detected tags in the image.
		#most images have zero (len(file_data)=0) or one (len(file_data)=1) tags.
		#The dictionaries have the following keys:
		#	'tag_family', 'pose_R', 'homography', 'center', 'corners', 'tag_id', 'decision_margin', 'hamming', 'pose_t', 'pose_err'
		#All of the arrays have been converted to numpy arrays.

		#DO STUFF WITH THE TAG DATA HERE...


	else:
		#print('No tags detected')
		a = 1


'''
#print(hist)
# sns.distplot(histogram,kde=False,hist_kws={'align':'mid'})
plt.hist(histogram,bins=[-0.25,0.25,0.75,1.25,1.75,2.25,2.75,3.25])
#plt.hist(histogram,bins=[-0.5,0.5,0.5,1.5,1.5,2.5,2.5,3.5])
plt.xlabel("Number of landmarks")
plt.ylabel("Number of Timesteps")
plt.show()




'''
#print(len(y_landmark))
#plt.plot(x_landmark,y_landmark, 'ob')
#plt.plot(x_r_landmark,y_r_landmark, 'ok')
#plt.plot(x_landmark[-2],y_landmark[-2], 'ob')
#plt.plot(0,0, 'or')
#plt.show()