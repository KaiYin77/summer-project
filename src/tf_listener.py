import roslib
# roslib.load_manifest('learning_tf')
import rospy
import tf 
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2

rospy.init_node('velodyne_points_to_map_frame', anonymous=True)

def callback(scan):
    '''
    Do transformation
    '''
    pass

if __name__== '__main__':
    
    listener = tf.TransformListener()
    rospy.Subscriber("/velodyne_points", PointCloud2, callback) 

    rate = rospy.Rate(1.0)
    listener.waitForTransform("/map", "/velodyne", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/velodyne', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()