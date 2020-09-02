#!/usr/bin/env python
import rospy
from apriltag_ros.msg import  AprilTagDetection
from apriltag_ros.msg import AprilTagDetectionArray
from ekf_slam_msgs.msg import EdgesStamped 
from ekf_slam_msgs.msg import OdomEdge 
from ekf_slam_msgs.msg import LandmarkEdge 
from ekf_slam_msgs.msg import tf_struct 
from nav_msgs.msg import Odometry 
import numpy as np
import math as m
import tf2_ros 
import tf2_geometry_msgs 
import Queue 

#Data:
#   Retrieve ros parameters:
dataRate=rospy.get_param('/front_end_node/dataRate',20)
linearUpdate=rospy.get_param('/front_end_node/linearUpdate',0.001)
angularUpdate=rospy.get_param('/front_end_node/angularUpdate',0.001)
input_queue_size=rospy.get_param('/front_end_node/input_queue_size',40)
tag_detection_threshold=rospy.get_param('/front_end_node/tag_detection_threshold',6)


#   Global queues:
global TAGS_QUEUE
TAGS_QUEUE=Queue.Queue(input_queue_size)   #Perceived tags queue

global ODOM_QUEUE   
ODOM_QUEUE=Queue.Queue(input_queue_size)   #Odometry queue

#   Global counters:
global TF_COUNTER
TF_COUNTER=0
global COUNT
COUNT=0

#Functions:
def odom_callback(data):
    """Put received odometry in the odom queue"""
    if not ODOM_QUEUE.full():
        ODOM_QUEUE.put(data)
    else:
        rospy.logwarn("Odom queue full, detection deprecated. Check if dataRate param match odom publication rate or add queues size")

def tags_callback(data):
    """Put received tags in the tags queue"""
    if not TAGS_QUEUE.full():
        TAGS_QUEUE.put(data)
    else:
        rospy.logwarn("Tags queue full, detection deprecated. Check if dataRate param match odom publication rate or add queues size")

                         
def retrieve_odom(Tx_queue):
    """Retrieve T(0,xi) data class from odometry message queue and insert associated tf_class to Tx_queue"""
    global COUNT
    while (not Tx_queue.full()) and (not ODOM_QUEUE.empty()):
                odom=ODOM_QUEUE.get()
                #Filling odometry transformation matrices
                covariance=[odom.twist.covariance[0],odom.twist.covariance[5],odom.twist.covariance[30],odom.twist.covariance[35]] 
                v=odom.twist.twist.linear.x
                w=odom.twist.twist.angular.z
                #Append tf dataclass to queue
                tf=tf_struct()
                tf.stamp=odom.header.stamp
                tf.seq=COUNT
                tf.v=v
                tf.w=w
                tf.covariance=covariance
                Tx_queue.put(tf)
                COUNT+=1

def retrieve_tags(Tm_queue,nbr_tag=2):
    """Retrieve T(xi,mk) topic from apriltag message queue"""
    nbr=0
    while (not Tm_queue.full()) and (not TAGS_QUEUE.empty()):
                nbr+=1
                tags=TAGS_QUEUE.get()
                Tm_queue.put(tags)
                if nbr==nbr_tag:
                    break

def synchronize(Tx_queue,Tm_queue,tf_queue):
    """Synchronize odom and tags queues"""
    #If two odom dataclasses available:
    if Tx_queue.full():
        #Retrieve tf1 and tf2 data class
        tf1=Tx_queue.get()
        tf2=Tx_queue.get()
        #Synchronisation
        out_sync_count=0
        tags_deprecated=0
        mid_timestamp=tf1.stamp + (tf2.stamp - tf1.stamp)*0.5 #mid_timestamp: middle timestamp of the two odom dataclasses timestamps
        #While tags get deprecated due to out of sync
        while True:
            out_sync=0
            #Wait for tags
            tries=0
            tags_detected=1
            global dataRate
            rate = rospy.Rate(dataRate*10)
            while Tm_queue.empty():
                tries+=1
                retrieve_tags(Tm_queue,1) #Retry retrieving tags
                rate.sleep()
                if tries > 20:
                    rospy.loginfo("No tags data received in %f  s, no visible tags considered",(2.0/dataRate))
                    tags_detected=0
                    break
 
            if tags_detected:
                rospy.logdebug("Synchronize: Tags detected")
                tags=Tm_queue.get()
                if tags.header.stamp < tf1.stamp:
                        out_sync_count+=1
                        out_sync=1
                        retrieve_tags(Tm_queue) #Retrieving new tags
                        rospy.logwarn("Tags deprecated, odometry out of sync: oldest available odometry at %s and oldest tags availble at %s. pose x:%d",tf1.stamp,tags.header.stamp,tf1.seq)
                        if out_sync_count==5:
                            rospy.logfatal("Continuous out of sync!")
                            exit(1)

                elif tags.header.stamp <= tf2.stamp: 
                        #Interpolation:
                        #Covariance?
                        rospy.logdebug("Synchronize: tf1 of %s interpolated with tags of %s. Pose x:%d",tf1.stamp,tags.header.stamp,tf1.seq)
                        dt_tag=tags.header.stamp-tf1.stamp
                        dt=tf2.stamp-tf1.stamp
                        ratio=dt_tag/dt
                        tf1.stamp=tags.header.stamp
                        X1=np.array([tf1.v,tf1.w])
                        X2=np.array([tf2.v,tf2.w])
                        X1=np.vstack(X1)
                        X2=np.vstack(X2)
                        R1=np.matrix([[tf1.covariance[0],tf1.covariance[1]],[tf1.covariance[2],tf1.covariance[3]]])
                        R2=np.matrix([[tf2.covariance[0],tf2.covariance[1]],[tf2.covariance[2],tf2.covariance[3]]])
                        R1_inv=np.linalg.inv(R1)
                        R2_inv=np.linalg.inv(R2)
                        R=np.linalg.inv((1-ratio)*R1_inv+ratio*R2_inv)
                        X=(1-ratio)*np.dot(R1_inv,X1)+ratio*np.dot(R2_inv,X2)
                        X=np.dot(R,X)
                        tf1.v=X[0]
                        tf1.w=X[1]
                        tf1.covariance=[R[0,0],R[0,1],R[1,0],R[1,1]]
                        tf1.tags=tags
                elif not Tm_queue.empty():    
                        #Re-insert tags in Tm_queue following  the correct order
                        rospy.logdebug("Synchronize: Reinsert tags of %s. Pose x:%d",tags.header.stamp,tf1.seq)
                        tags2=Tm_queue.get()
                        Tm_queue.put(tags)
                        Tm_queue.put(tags2)
                else:   
                        rospy.logdebug("Synchronize: Reinsert tags of %s. Pose x:%d",tags.header.stamp,tf1.seq)
                        Tm_queue.put(tags)
            #Break while true
            if out_sync==0:
                rospy.logdebug("Synchronize: Finished!")
                break
        Tx_queue.put(tf2)
        tf_queue.put(tf1)

TAG_DETECT=0
def update_tr_queue(queue):
    """Keep or not the tf2 of tf_queue according to the linear/angularUpdate params"""
    global TF_COUNTER
    global TAG_DETECT
    if queue.full():
        #Calculate angular and linear steps
        tf1=queue.get()
        tf2=queue.get()
        dv=abs(tf2.v-tf1.v)
        dw=abs(tf2.w-tf1.w)
        #Test:
        if dv>=linearUpdate or dw>=angularUpdate :
            tf1.seq=TF_COUNTER
            TF_COUNTER+=1
            queue.put(tf1)
            queue.put(tf2)
            TAG_DETECT=0
        elif (not TAG_DETECT) and len(tf2.tags.detections)!=0:
            tf1.seq=TF_COUNTER
            TF_COUNTER+=1
            queue.put(tf1)
            queue.put(tf2)
            TAG_DETECT=1
        else:
            queue.put(tf1)
            
def Transform_pose(input_pose, to_frame): 
    """Transform pose from one frame to another"""   
    tf_buffer = tf2_ros.Buffer(rospy.Duration(2))
    listener = tf2_ros.TransformListener(tf_buffer)
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
            raise


def transform_tags(tf,output):
    """Publish tags transformations using tags apriltag msg"""
    tags=tf.tags
    out_tags=[]
    for detection in tags.detections:
        tag=LandmarkEdge()
        tag.id=detection.id[0]  
        #Note that tag detection is made according to a camera frame with a orientation convention. Transformation needs to be done:
        out_pose=Transform_pose(detection.pose,"base_link")
        tag.r=m.sqrt(out_pose.position.x**2+out_pose.position.y**2)
        if tag.r < tag_detection_threshold:
            if  out_pose.position.y<1e-3:
                tag.phi=0
            else:
                tag.phi=m.atan(out_pose.position.x/out_pose.position.y)
            tag.covariance=[0.0016*(tag.r)**2,0,0,0.0123*(tag.phi)**2]  # 20cm standard deviation on r at 5m, 5 degrees on phi at pi/4
            out_tags.append(tag)
            rospy.logdebug("Transform: tag %d associated to x%d published",tag.id,tf.seq)
    output.tags=out_tags

def transform_odom(tf1,output):
    """Publish odometry transformation using two Tx_class dataclasses"""
    output.header.stamp=tf1.stamp
    i=tf1.seq
    j=i+1
    output.var="x"+str(i)+"x"+str(j)
    #OdomEdge filling
    output.E.v=tf1.v 
    output.E.w=tf1.w 
    output.E.covariance=[tf1.covariance[0],tf1.covariance[1],0,tf1.covariance[2],tf1.covariance[3],0,0,0,0.0003046] #1 degree standard deviation on final pose
    rospy.logdebug("Transform: OdomEdge %d filled to be published",i)

def transform(tf_queue,pub):
    """Compute output: odometry and tags transformations using tf_queue """
    if tf_queue.full():
        tf1=tf_queue.get()
        tf2=tf_queue.get()
        output=EdgesStamped()
        dt=tf2.stamp-tf1.stamp
        output.dt=dt.toSec()
        #Odometry transformation T(xi,xj)
        transform_odom(tf1,output)
        #Tags transformation
        transform_tags(tf1,output)
        #Publish Edges:
        pub.publish(output)
        #Reinsert tf2:
        tf_queue.put(tf2)

def front_end_node():
        rospy.init_node('front_end_node', anonymous=True)

        #Pub/Sub
        rate = rospy.Rate(dataRate)
        pub = rospy.Publisher('/transformations', EdgesStamped, queue_size=15)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tags_callback)
        rospy.Subscriber("/odom", Odometry, odom_callback)

        #Init variables:
        #T(0,xi) queue (Odometry) 
        Tx_queue=Queue.Queue(2)
        #T(xi,mk) queue (landmarks) 
        Tm_queue=Queue.Queue(2)
        #Transformations queue (tf_class) 
        tf_queue=Queue.Queue(2)
        
        rospy.loginfo("Front end started!")
        while not rospy.is_shutdown():
                rospy.logdebug("Retrieving data")
                retrieve_odom(Tx_queue)
                retrieve_tags(Tm_queue)
                rospy.logdebug("Synchronizing data")
                synchronize(Tx_queue,Tm_queue,tf_queue)     
                rospy.logdebug("Update data")
                update_tr_queue(tf_queue)
                rospy.logdebug("Transform data")
                transform(tf_queue,pub)   
                    
                rate.sleep()


if __name__ == '__main__':
    try:
        front_end_node()

    except rospy.ROSInterruptException:
        pass

