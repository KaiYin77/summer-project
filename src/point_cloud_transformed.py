import roslib
import rospy
import tf 
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2

rospy.init_node('velodyne_points_to_map_frame', anonymous=True)
transform = geometry_msgs.msg.TransformStamped()                           

def pointcloud_callback(scan):

    '''
    Do transformation
    '''
    print('transform:', transform)

if __name__== '__main__':
    
    listener = tf.TransformListener()
    rospy.Subscriber("/velodyne_points", PointCloud2, callback) 

    rate = rospy.Rate(1.0)
    listener.waitForTransform("/map", "/velodyne", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
            listener.lookupTransform('/map', '/velodyne', transform)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
