import rospy
import numpy
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
covariance = []
def amcl_call(data):
    # print(data.pose.covariance[0])
    covariance.append(data.pose.covariance)

def main():
    rospy.init_node('plot',anonymous=True)
    rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,amcl_call)
    rospy.spin()
    # map = rospy.wait_for_message('/map',OccupancyGrid)
if __name__ == "__main__":
    main()
    covariance = list(numpy.concatenate(covariance).flat)  
    print(covariance)
    plt.plot(covariance)
    plt.xlabel('Iteration')
    plt.ylabel('covariance')
    plt.title('covariance plot')
    plt.legend()
    plt.show()