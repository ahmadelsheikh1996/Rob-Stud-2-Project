#!/usr/bin/env python

import numpy as np
import time
import cv2
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from particle_filter import ParticleFilter
import functions
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import ObjectCount
global stop1 # stop on the stop sign
stop1= False
global stop2 # lanes stopping
stop2=False
global move_left
move_left = False
global stop_cup # stop when detecting a cup
stop_cup=False
#global cup_detect_time
#cup_detect_time = 0
#global stop_detect_time
#stop_detect_time = 0
#global waiting_time

# Mask thresholds (HSV) for blue color
mask_lower_threshold_solid = np.array([40,40,0])
mask_upper_threshold_solid = np.array([150, 255,150])


# The commented code is part of my project. The pink mask is for the pink dashed lanes. The cup_detect_time and stop_detect_time are variables to store the time of detection of these objects. The waiting time is used so I can check again after some time to see if the object is still in its place. For stop sign: I can do [ if waiting time > 0.8: stop1 = False]. I also need to use it to check for pedestrains again after 1 or 2 seconds on the crossroad. So, if pedestrian is detected, stop. Then, check after 5 seconds for the pedestrian. After that, if box.class != 'cup', move again.
# The commented printing commands are used to check if the code is running well, or to print some values to monitor the progress of the project.


# trim configuration in the command window
###rosrun dynamic_reconfigure dynparam set /duckiebot/diff_drive trim 0.067



#pink:
#mask_lower_threshold_dashed= np.array([140,40,40])
#mask_upper_threshold_dashed = np.array([180, 255, 255])

# Lane width
lane_width = 0.3

def box_check(data):
#    global cup_detect_time
#    global stop_detect_time
    global stop1
    global stop_cup
    global move_left
#    global waiting_time
#    print('object detection running')
    for box in data.bounding_boxes:
	if box.Class == 'cup':
#		cup_detect_time = rospy.Time.now().to_sec()
		print('Cup detected')
#		print (cup_detect_time)
		stop_cup = True

		
	elif box.Class == 'stop sign':
		stop1=True
#		stop_detect_time = rospy.Time.now().to_sec()
		print('Stop sign detected')
#		print (stop_detect_time)
		

	elif box.Class == 'road sign':
		print('Left arrow')
		move_left = True


def object_count(data_count):
	global stop1
	global stop_cup
	global objects_count
	objects_count = data_count.count
	print(objects_count)
	if objects_count==0:
		stop1=False
		stop_cup=False
		



def plot_line_segments(image, line_segments):
    # Draw lines on image
    for line_segment in line_segments:
        u1, v1, u2, v2 = line_segment
        cv2.line(image, (int(u1), int(v1)), (int(u2), int(v2)), (255, 0, 255), 1, cv2.LINE_AA)

    return image


def plot_lanes(image, left_lane, right_lane, horiz_lane):
    global stop2
    global stop_cup
    if left_lane is not None:
        [u1, v1, u2, v2] = left_lane
        cv2.line(image, (int(u1), int(v1)), (int(u2), int(v2)), (0, 0, 255), 2, cv2.LINE_AA)

    if right_lane is not None:
        [u1, v1, u2, v2] = right_lane
        cv2.line(image, (int(u1), int(v1)), (int(u2), int(v2)), (255, 0, 0), 2, cv2.LINE_AA)


   ###horiontal lane:
    if horiz_lane is not None:
        [u1, v1, u2, v2] = horiz_lane
        cv2.line(image, (int(u1), int(v1)), (int(u2), int(v2)), (100, 100, 0), 2, cv2.LINE_AA)
    	if stop_cup==True:
		stop2=True
    	else:
		stop2=False

    return image


def estimate_pose(left_lane_image, right_lane_image):
    # Estimate pose from a single observation

    global lane_width

    if right_lane_image is None:
        # Only left lane is detected
        left_lane_ground = functions.line_image2ground(left_lane_image)
        left_slope, left_intercept = functions.line_points2slope_intercept(left_lane_ground)

        y_expected = (lane_width / 2.) - left_intercept

        left_angle = np.arctan(left_slope)
        phi_expected = -left_angle

    elif left_lane_image is None:
        # Only right lane detected
        right_lane_ground = functions.line_image2ground(right_lane_image)
        right_slope, right_intercept = functions.line_points2slope_intercept(right_lane_ground)

        y_expected = (-lane_width / 2.) + right_intercept

        right_angle = np.arctan(right_slope)
        phi_expected = -right_angle

    else:
        # Both lanes are detected
        left_lane_ground = functions.line_image2ground(left_lane_image)
        right_lane_ground = functions.line_image2ground(right_lane_image)

        # Determine Duckiebot pose from observation
        left_slope, left_intercept = functions.line_points2slope_intercept(left_lane_ground)
        right_slope, right_intercept = functions.line_points2slope_intercept(right_lane_ground)

        # Expected y position is the deviation from the centre of the left and right intercepts
        y_expected = -(left_intercept + right_intercept) / 2.

        # Convert slopes to angles
        left_angle = np.arctan(left_slope)
        right_angle = np.arctan(right_slope)

        # Expected angle is the negative of the average of the left and right slopes
        phi_expected = -((left_angle + right_angle) / 2.)

    return [y_expected, phi_expected]


class LaneFollower:
    def __init__(self):
        # CV bridge
        self.bridge = CvBridge()

        # Particle filter
#        global lane_width
#        self.particle_filter = ParticleFilter(1000, -lane_width / 2, lane_width / 2, -np.pi / 8., np.pi / 8.,
 #                                             lane_width)

  #      self.prev_v = 0.
  #      self.prev_omega = 0.
 #       self.prev_time = rospy.Time.now()
#
#        self.steps_since_resample = 0  


        # Publisers
        self.mask_pub = rospy.Publisher('/mask', Image, queue_size=1)
        self.edges_image_pub = rospy.Publisher('/edges_image', Image, queue_size=1)
        self.line_segments_image_pub = rospy.Publisher('/line_segments_image', Image, queue_size=1)
        self.lanes_image_pub = rospy.Publisher('/lanes_image', Image, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Image subscriber
        self.image_sub = rospy.Subscriber('/duckiebot/camera_node/image/rect', Image, self.image_callback)

    def stop(self):
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

    def lane_detection(self, image_bgr,mask_lower_threshold, mask_upper_threshold, issolid):
        # Convert to HSV
        image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

        # Mask
        mask = cv2.inRange(image_hsv, mask_lower_threshold, mask_upper_threshold)

        # Publish mask image
        if self.mask_pub.get_num_connections() > 0:
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, encoding='passthrough'))

        # Canny edge detection
        edges = cv2.Canny(mask, 200, 400)

        # Clear the top half of the edges image
        edges[0:int(float(edges.shape[0]) / 2.), :] = 0.

        # Publish edges image
        if self.edges_image_pub.get_num_connections() > 0:
            self.edges_image_pub.publish(self.bridge.cv2_to_imgmsg(edges, encoding='passthrough'))

        # Detect line segments
        line_segments_tmp = cv2.HoughLinesP(edges, 1, np.pi / 180., 10, None, 8, 4)

        if line_segments_tmp is None:
            print('No line segments detected')
            return [None, None, None]

        # Remove extra array layer from line_segments_tmp
        line_segments = []

        for line_segment in line_segments_tmp:
            line_segments.append(line_segment[0])

        # Publish line segments image
        if self.line_segments_image_pub.get_num_connections() > 0:
            line_segments_image = plot_line_segments(image_bgr, line_segments)
            self.line_segments_image_pub.publish(self.bridge.cv2_to_imgmsg(line_segments_image, encoding='bgr8'))

        # Combine line segments
        [left_lane, right_lane, horiz_lane] = functions.average_slope_intercept(line_segments)

        if self.lanes_image_pub.get_num_connections() > 0:
            if issolid:
            	lanes_image = plot_lanes(image_bgr, left_lane, right_lane, horiz_lane)
            else:
	    	lanes_image = plot_lanes(image_bgr, left_lane, right_lane, None)
            self.lanes_image_pub.publish(self.bridge.cv2_to_imgmsg(lanes_image, encoding='bgr8'))

        return [left_lane, right_lane, horiz_lane]

    def image_callback(self, image_msg):
        global stop1
	global stop_cup
        global stop2
#	global cup_detect_time
#	global stop_detect_time
	global move_left
        global mask_lower_threshold_solid, mask_upper_threshold_solid
#	global waiting_time
 #       global mask_lower_threshold_dashed, mask_upper_threshold_dashed
        # Convert image message to OpenCV image
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except CvBridgeError as e:
            print(e)

        # Lane detection

        #solid        
        [left_lane_image_solid, right_lane_image_solid, horiz_lane_image_solid] = self.lane_detection(image_bgr,mask_lower_threshold_solid, mask_upper_threshold_solid, True)    


        #dashed
  #      [left_lane_image_dashed, right_lane_image_dashed, horiz_lane_image_dashed] = self.lane_detection(image_bgr,mask_lower_threshold_dashed, mask_upper_threshold_dashed, False)    


        # Stop if no lanes are detected
        if left_lane_image_solid is None and right_lane_image_solid is None:
          #  stop2=True
#	    print('no left or right lanes')
	    return



# In this part, I have to check for the dashed and solid lines. So, I have to check which of them is the right lne and which is the left and use them in the position estimate. If move left== True: Then, dashed line has to be the right line while moving and the solid left will be the left line. If not, the dashed line should be the left line, and the solid right line has to be the right line when moving.
			
# I enter this code before the position estimate, and I use it in the pose estimate so the duckiebot knows in which lane to move



        # Estimate position from single observation
        [y, phi] = estimate_pose(left_lane_image_solid, right_lane_image_solid)
	
	y = y + 0.02

        # # Estimate pose with particle filter
        # if (self.steps_since_resample >= 10):
        #     self.particle_filter.resample_particles()
        #     self.steps_since_resample = 0
        #
        # t = (rospy.Time.now() - self.prev_time).to_sec()
        #[y, phi] = self.particle_filter.step(self.prev_v, self.prev_omega, t, left_lane_image, right_lane_image)


        # Determine control input
	v = 0.1
        omega = -2.25 * y + -3.75 * phi  # Proportional only


        # Publish control input
        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = omega


	if stop1==True or stop2==True:
        	self.stop()
	else:
#		if move_left==True:
			#move left control. I didn't do this part yet.
#		else: 
			#normal control which is: self.cmd_vel_pub.publish(cmd_vel)
		self.cmd_vel_pub.publish(cmd_vel)
#        	print('y = ', y)
#        	print('phi = ', phi)
#	        print('v = ', v)
#	        print('omega = ', omega)
		return


        # Save control input for particle filter
        #self.prev_v = v
        #self.prev_omega = omega
        #self.prev_time = rospy.Time.now()
        #self.steps_since_resample += 1



if __name__ == '__main__':
    rospy.init_node('lane_follower')
    data = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, box_check)
    data_count = rospy.Subscriber("/darknet_ros/found_object", ObjectCount, object_count)
    lane_follower = LaneFollower()
    rospy.on_shutdown(lane_follower.stop)

    rospy.spin()
