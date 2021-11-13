import rospy
import tf.transformations as tft
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64

t = Twist()
pub = tf.TransformBroadcaster()

def callback(data):
	rospy.loginfo("NODE MOVE %s",data.data)

def main():
	rospy.init_node('pub_map_odom', anonymous=True)
	#quit = False
	rate = rospy.Rate(10)
