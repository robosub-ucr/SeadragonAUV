#!/usr/bin/env python

import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import time
from collections import defaultdict
from io import StringIO
from PIL import Image
import cv2
import numpy as np
import rospy
from std_msgs.msg import Bool, Float64, Int16

sys.path.append("..")

cap = cv2.VideoCapture(2)

##------------------------------ MODEL CONFIGURATION ------------------------------------------#

# ## Object detection imports
from imports.utils import label_map_util
from imports.utils import visualization_utils as vis_util

# What model to download.
MODEL_NAME = 'imports/buoy_graph'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('imports/data', 'object-detection.pbtxt')

# Number of Classes
NUM_CLASSES = 2

# Load a (frozen) Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')

# Load label map
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Helper code
def load_image_into_numpy_array(image):
  (im_width, im_height) = image.size
  return np.array(image.getdata()).reshape(
      (im_height, im_width, 3)).astype(np.uint8)

##-------------------------- END MODEL CONFIGURATION ----------------------------------------#


##------------------------------ TEST IMAGE PATHS -------------------------------------------#

# If you want to test the code with your images, just add path to the images to the TEST_IMAGE_PATHS.
PATH_TO_TEST_IMAGES_DIR_1 = 'imports/test_images/lava'
PATH_TO_TEST_IMAGES_DIR_2 = 'imports/test_images/flat'

LAVA_PATHS = [ os.path.join(PATH_TO_TEST_IMAGES_DIR_1, 'lavaimage{}.png'.format(i)) for i in range(319, 637) ]
FLAT_PATHS = [ os.path.join(PATH_TO_TEST_IMAGES_DIR_2, 'flatimage{}.png'.format(i)) for i in range(310, 596) ]

##------------------------------- END TEST IMAGES -------------------------------------------#



# Vision Enable Subscribers


# Object Location Publishers
task_publisher     	= rospy.Publisher('/task_detected', Int16, queue_size=10)

buoy_flat_x_pub		= rospy.Publisher('/buoy_flat_x', Float64, queue_size=10)
buoy_flat_y_pub		= rospy.Publisher('/buoy_flat_y', Float64, queue_size=10)
buoy_flat_area_pub	= rospy.Publisher('/buoy_flat_area', Float64, queue_size=10)

buoy_tri_x_pub		= rospy.Publisher('/buoy_triangle_x', Float64, queue_size=10)
buoy_tri_y_pub		= rospy.Publisher('/buoy_triangle_y', Float64, queue_size=10)	
buoy_tri_area_pub 	= rospy.Publisher('/buoy_triangle_area', Float64, queue_size=10)

buoy_tri_x 		= Float64()
buoy_tri_y 		= Float64()
buoy_tri_area 		= Float64()

buoy_flat_x 		= Float64()
buoy_flat_y 		= Float64()
buoy_flat_area 		= Float64()

task_detected		= Int16()

GATE_TASK		= 1
TORPED_TASK		= 2

class vision():
		
	def __init__(self):
		self.enable 		= False
		self.taskCompletedList  = [False, False]
		self.GATE		= 0
		self.BUOY		= 1

	def set_enable(self, x):
		self.enable = x
	
	def get_enable(self):
		return self.enable

	def check_task_completed(self,task):
		return self.taskCompletedList[task]

	def update_task_completed(self,task):
		self.taskcompletedList[task] = True
	
	def detect(self,image_np):
		
      		image_np_expanded = np.expand_dims(image_np, axis=0)
      		image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
      		# Each box represents a part of the image where a particular object was detected.
      		boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
      		# Each score represent how level of confidence for each of the objects.
      		# Score is shown on the result image, together with the class label.
      		scores = detection_graph.get_tensor_by_name('detection_scores:0')
      		classes = detection_graph.get_tensor_by_name('detection_classes:0')
      		num_detections = detection_graph.get_tensor_by_name('num_detections:0')
      		# Actual detection.
      		(boxes, scores, classes, num_detections) = sess.run(
          	[boxes, scores, classes, num_detections],
          	feed_dict={image_tensor: image_np_expanded})
      		# Visualization of the results of a detection.
      		vis_util.visualize_boxes_and_labels_on_image_array(
          	   image_np,
          	   np.squeeze(boxes),
          	   np.squeeze(classes).astype(np.int32),
          	   np.squeeze(scores),
          	   category_index,
          	   use_normalized_coordinates=True,
          	   line_thickness=8)
		
		# Iterate through boxes and look for Buoys, Gate, Torpedo?
      		for i,b in enumerate(boxes[0]):

			# Flat Buoy Detected
      			if classes[0][i] == 1:
				# Check if Buoy Task has been completed
				if self.check_task_completed(self.BUOY) == False:
	  				# Check if score is high enough to be actual detection 
	  				if scores[0][i] >= 0.9: 
						mid_x = (boxes[0][i][1]+boxes[0][i][3])/2
         	 				mid_y = (boxes[0][i][0]+boxes[0][i][2])/2
						print "Flat Buoy at: " + str(mid_x*400) + ", " + str(mid_y*300)
					
						# Let Master Know Buoy Task was detected
						
						# Publish Area data
						buoy_flat_area.data = (((boxes[0][i][3] - boxes[0][i][1]) * 400) * ((boxes[0][i][2] - boxes[0][i][0]) * 300 )) / 120000
						buoy_flat_area_pub.publish(buoy_flat_area)						

						# Publish Location data
						buoy_flat_x.data = mid_x * 400
						buoy_flat_x_pub.publish(buoy_flat_x)
						buoy_flat_y.data = mid_y * 300
						buoy_flat_y_pub.publish(buoy_flat_y)	
			# Lava Buoy Detected
      			if classes[0][i] == 2:
				# Check if Buoy Task has been completed
				if self.check_task_completed(self.BUOY) == False:
	  				# Check if score is high enough to be actual detection 
	  				if scores[0][i] >= 0.9: 
						mid_x = (boxes[0][i][1]+boxes[0][i][3])/2
         	 				mid_y = (boxes[0][i][0]+boxes[0][i][2])/2
						print "Lava Buoy at: " + str(mid_x*400) + ", " + str(mid_y*300)
						# Publish Location data
						buoy_tri_x.data = mid_x * 400
						buoy_tri_x_pub.publish(buoy_tri_x)
						buoy_tri_y.data = mid_y * 300
						buoy_tri_y_pub.publish(buoy_tri_y)
		return image_np		



if __name__ == '__main__': 
	
	rospy.init_node('vision', anonymous=False)

	dragonVision = vision()
	
	rate = rospy.Rate(20)

	with detection_graph.as_default():
  		with tf.Session(graph=detection_graph) as sess:
			while not rospy.is_shutdown():
				#for image_path in FLAT_PATHS:
      					#image_np = cv2.imread(image_path)
      					ret, image_np = cap.read()
					
					# Run Detection process
					image_np = dragonVision.detect(image_np)
					
					# Detection Visualization	
					cv2.imshow('object detection', cv2.resize(image_np, (400,300)))
     					if cv2.waitKey(25) & 0xFF == ord('q'):
        					cv2.destroyAllWindows()	
						break
			rate.sleep()
		
