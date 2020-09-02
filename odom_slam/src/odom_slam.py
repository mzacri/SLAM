#!/usr/bin/env python
import rospy
from geometry_msgs.msg import  PoseArray
from geometry_msgs.msg import  Pose
from apriltag_ros.msg import  AprilTagDetection
from apriltag_ros.msg import AprilTagDetectionArray 

import tf2_ros
import tf2_geometry_msgs
import Queue    

numberTags=rospy.get_param('/odom_slam/number_tags',19)
mapUpdateRate=rospy.get_param('/odom_slam/map_updtae_rate',10)

global perceived_tags_queue
perceived_tags_queue=Queue.Queue(2)
def Transform_pose(input_pose, to_frame,tf_buffer):
	pose_stamped = tf2_geometry_msgs.PoseStamped()
	pose_stamped.pose = input_pose.pose.pose
	pose_stamped.header.frame_id = input_pose.header.frame_id
	pose_stamped.header.stamp = input_pose.header.stamp
	pose_stamped.header.seq = input_pose.header.seq

	try:
		# ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
		output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
		return output_pose_stamped.pose

	except ( tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		raise
        except tf2_ros.LookupException:
                rospy.loginfo("Tag passed due to looKupException")
                raise
def Callback(data):
	if perceived_tags_queue.full():
		flushedtag=perceived_tags_queue.get()	   
	perceived_tags_queue.put(data)

def OdomSLAM_node():
	rospy.init_node('odom_slam', anonymous=True)
        #Init tf buffer
        tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
	listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(2)
	#Pub/Sub
        rate = rospy.Rate(mapUpdateRate)
	pub = rospy.Publisher('localised_tags', PoseArray, queue_size=10)
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, Callback)
        #Init variables:
        pose=Pose()
        localised_tags=PoseArray()
        localised_tags.poses=numberTags*[pose]
        localised_tags.header.frame_id="odom"
        rospy.loginfo("Tags detection started!")
	while not rospy.is_shutdown():
		while not perceived_tags_queue.empty():
                        tags=perceived_tags_queue.get()			
			for detection in tags.detections:
				i=detection.id[0]
                                rospy.loginfo("Tag %d detected!",i)
                                odomPose=Transform_pose(detection.pose, "odom",tf_buffer)
				localised_tags.poses[i]=odomPose
                        localised_tags.header.stamp=rospy.Time.now()
                        localised_tags.header.seq+=1
                        try:
			    pub.publish(localised_tags)
                        except AttributeError:
                            rospy.loginfo("AttributeError Passed!")
                            pass
                pass
		rate.sleep()

if __name__ == '__main__':
    try:
        OdomSLAM_node()

    except rospy.ROSInterruptException:
        pass

