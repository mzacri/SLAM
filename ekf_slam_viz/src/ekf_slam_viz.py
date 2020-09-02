#!/usr/bin/env python
import rospy
from ekf_slam_msgs.msg import MeanCovariance
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import sqrt
from math import atan2
from math import pi

numberTags=rospy.get_param('/odom_slam/number_tags',19)

TRAJECTORY=[]
def publish_line_marker(x,y):
    """Publish trajectory by adding (x,y) point to the end of the trajectory"""
    global TRAJECTORY
    #Fill odom point
    point=Point()
    point.x=x
    point.y=y
    point.z=0.0
    TRAJECTORY.append(point)
    #Fill line marker
    marker=Marker()
    marker.header.frame_id = "odom";
    marker.header.stamp = rospy.Time.now();
    marker.ns = "odom";
    marker.id = 0;
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;
    marker.color.a = 1.0; 
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.lifetime=0;
    marker.points=TRAJECTORY
    vis_pub.publish( marker );

def publish_odom_covariance_marker(x,y,cov):
    """publish an elipse marker to represent covariance of odometry at (x,y)"""
    #Calculate covariance ellipse params:
    l1=(cov[0]+cov[3])/2
    l2=(cov[0]-cov[3])/2
    m1=l1+sqrt(l2**2+cov[1])
    m2=l1-sqrt(l2**2+cov[1])
    if cov[1]<1e-5 and cov[0] >= cov[3]:
        theta=0
    elif cov[1]<1e-5 and cov[0] < cov[3]:  
        theta=pi/2
    else:
        theta=atan2(m1-cov[0],cov[1])
    quat_w=cos(theta/2)
    quat_z=sin(theta/2)
    #Fill line marker
    marker=Marker()
    marker.header.frame_id = "odom";
    marker.header.stamp = rospy.Time.now();
    marker.ns = "odom_cov";
    marker.id = 0;
    marker.type = marker.SPHERE;
    marker.action = marker.ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = quat_z;
    marker.pose.orientation.w = quat_w;
    marker.scale.x = sqrt(m1);
    marker.scale.y = sqrt(m2);
    marker.scale.z = 0.0;
    marker.color.a = 1.0; 
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime=0;
    vis_pub.publish( marker );



def publish_tag_covariance_marker(tag_id,x,y,cov):
    """publish an elipse marker to represent covariance at (x,y)"""
    #Calculate covariance ellipse params:
    l1=(cov[0]+cov[3])/2
    l2=(cov[0]-cov[3])/2
    m1=l1+sqrt(l2**2+cov[1])
    m2=l1-sqrt(l2**2+cov[1])
    if cov[1]<1e-5 and cov[0] >= cov[3]:
        theta=0
    elif cov[1]<1e-5 and cov[0] < cov[3]:  
        theta=pi/2
    else:
        theta=atan2(m1-cov[0],cov[1])
    quat_w=cos(theta/2)
    quat_z=sin(theta/2)
    #Fill line marker
    marker=Marker()
    marker.header.frame_id = "odom";
    marker.header.stamp = rospy.Time.now();
    marker.ns = "tag_cov";
    marker.id = tag_id;
    marker.type = marker.SPHERE;
    marker.action = marker.ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = quat_z;
    marker.pose.orientation.w = quat_w;
    marker.scale.x = sqrt(m1);
    marker.scale.y = sqrt(m2);
    marker.scale.z = 0.0;
    marker.color.a = 1.0; 
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime=0;
    vis_pub.publish( marker );



def publish_tag_marker(tag_id,x,y):
    """Publish tag tag_id marker at (x,y) coordinates"""
    #Fill line marker
    marker=Marker()
    marker.header.frame_id = "odom";
    marker.header.stamp = rospy.Time.now();
    marker.ns = "tags";
    marker.id = tag_id;
    marker.type = marker.CYLINDER;
    marker.action = marker.ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; 
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime=0;
    vis_pub.publish( marker );


    
def callback(data):
    ekf_out=data
    #Retrieve corrected mean-covariance: 
    len_mean=len(ekf_out.mean)/3
    mean=np.array(ekf_out.mean)
    covariance=np.array(ekf_out.covariance)
    try:
        mean=np.reshape(mean,(len_mean,3))
        covariance=np.reshape(covariance,(6*len_mean,6*len_mean)) 
    except:
        rospy.logerror("Recevied covariance or mean shapes inconsistents")
        raise

    ##Publish data:
    for i in range(len_mean):
        [x,y]=mean[i,0:2]
        cov=covariance[6*i:6*i+2,6*i:6*i+2]
        cov=cov.reshape(4).ravel()
        if i==0:
            publish_line_marker(x,y)
            publish_odom_covariance_marker(x,y,cov)
        else:
            publish_tag_marker(i,x,y)
            publish_tag_covariance_marker(i,x,y,cov)    


def ekf_slam_viz():
    rospy.init_node("ekf_slam_viz")
    #Pub/Sub:
    pub1=rospy.Publisher("odom_markers", Marker, queue_size=10)
    pub2=rospy.Publisher("tags_markers", Marker, queue_size=10)
    rospy.Subscriber("ekf_output", MeanCovariance, callback)
    rospy.loginfo("Ekf_slam viz started!")

    rospy.spin() 

if __name__ == '__main__':
    try:
        ekf_slam_viz()
    except rospy.ROSInterruptException:
        pass

