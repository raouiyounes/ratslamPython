
import os
import numpy as np
import cv2
from matplotlib import pyplot as plt
import pdb

class LearnToDetect:
	Descriptors=[]
	points_of_all_images=[]

	def __init__(self,path_of_db):
		self.path_of_db=path_of_db
		self.image_matrices=[]
		self.descriptors_of_entire_db=[]

	def read_images(self):
		path_of_objects="/home/younes/Images/ratslam_test/rospyy/object/"
		for image_name in os.listdir(self.path_of_db):

			self.image_matrices.append(cv2.imread(path_of_objects+image_name,1))
			
	def extract_surf(self):
		# threshold of the Hessian
		surf=cv2.ORB_create(400)
		for i in range(len(self.image_matrices)):
			#print "object's number : ", i 
		 	kp,des=surf.detectAndCompute(self.image_matrices[i],None)
		 	self.descriptors_of_entire_db.append([des,i])
		 	



class Recognition:
	#pdb.set_trace()
	def __init__(self,image_path):
		self.im_test_matrix=cv2.imread(image_path,1)
		self.keypoint=[]
		self.descriptor=[]

	def extract_features_of_test(self):
		surf=cv2.ORB_create()
		self.keypoint,self.descriptor=surf.detectAndCompute(self.im_test_matrix,None)
		print self.im_test_matrix
		plt.imshow(self.im_test_matrix)
		plt.show()
	def matching(self,des2_db):
		bf=cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)
		matches=bf.match(self.descriptor,des2_db)
		matches=sorted(matches,key=lambda x:x.distance)




class ComputeFeature:


	def apply_hessian():
		orb=cv2.ORB_create()


		




learn_object=LearnToDetect("/home/younes/Images/ratslam_test/rospyy/object")
learn_object.read_images()
learn_object.extract_surf()
#reco=Recognition("/home/younes/Images/testIm/test.jpg")
#reco.extract_features_of_test()









