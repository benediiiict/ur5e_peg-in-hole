#!/usr/bin/python3
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped

class Get_force_val():
  def __init__(self):
    rospy.init_node('wrench_subscriber', anonymous=True)
    # robotiq_ft_wrench                                      force_return
    rospy.Subscriber('/ft_sensor_topic', WrenchStamped, self.force_return)
    rospy.spin()
    self.ft_sensor_array = []

  def force_return(self,data):
    self.force_data = []
    self.force_data.append(data.wrench.force.x)
    self.force_data.append(data.wrench.force.y)
    self.force_data.append(data.wrench.force.z)
    print(self.force_data)
    return self.force_data

  def force_data(self,data):
        # rospy.loginfo(data)
    self.adjust = []
    self.adjust = np.array([data.wrench.force.x,
                            data.wrench.force.y,
                            data.wrench.force.z,
                            data.wrench.torque.x,
                            data.wrench.torque.y,
                            data.wrench.torque.z])
    # self.adjust[2] -= 11.898
    self.ft_sensor_array = self.adjust
    print(self.ft_sensor_array)  

if __name__ == '__main__':
  print("start")
  GF = Get_force_val()
  fforce_data = GF.force_return()
  # print(fforce_data)
