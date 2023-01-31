import rospy
from orbbec_camera.srv import SetBool
import sys
import time
def main():
    rospy.init_node('set_laser', anonymous=True)
    service = "/camera/set_laser"
    rospy.wait_for_service(service)
    try:
      start = time.time()
      request = rospy.ServiceProxy(service, SetBool)
      response = request(True)
      now = time.time()
      print("succes =  %s, time cost %s", response.success, (now - start))
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    rospy.spin()


if __name__ == '__main__':
    main()