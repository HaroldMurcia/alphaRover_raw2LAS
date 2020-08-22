import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import tf
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from datetime import date

class NodoPos(object):
  #%% Inizialization
  def __init__(self):
    self.ekf_pub = rospy.Publisher('/ekf2', Odometry, queue_size=10)
    #%% Encoder variables
    self.PPR = 6533.0
    self.radio = 0.07
    self.enc_toc = 0.0
    self.enc_Ts = 0.0
    self.enc_tic =  0.0
    self.enc_pulses_R1 = 0
    self.enc_pulses_L1 = 0
    self.flag_enco = True
    self.wr = 0.0
    self.wl = 0.0
    self.wr_1 = 0.0
    self.wl_1 = 0.0
    self.wr_2 = 0.0
    self.wl_2 = 0.0
    #%% Filter_encoders variables
    self.acce = 2.0
    self.wr_f_2 = 0.0
    self.wr_f_1 = 0.0
    self.wl_f_2 = 0.0
    self.wl_f_1= 0.0
    #%% IMU variables
    self.flag_imu = True
    self.euler1 = [0,0,0]
    self.imu_toc = 0.0
    self.imu_Ts = 0.0
    self.imu_tic = 0.0
    self.imu_pitch = 0.0
    self.imu_roll = 0.0
    self.imu_yaw = 0.0
    #%% Kalman variables
    # Matrix error of covariance
    self.kalman_P = np.matrix([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 100.0]])
    # Covariance matrix of process noise
    self.kalman_Q = np.matrix([[0.5, 0.0, 0.0],[0.0, 0.5, 0.0],[0.0, 0.0, 0.1]])
    # Covariance matrix of measurement noise
    self.kalman_R = 0.0265
    self.Yg = 1.2
    self.X_hat = np.matrix([[0.0],[0.0],[0.0]])
    #%% INIT
    self.today = date.today()
    rospy.loginfo("Starting Kalman node")
    rospy.Subscriber('/encoders', String, self.callEncoder)		#10 Hz
    rospy.Subscriber('/imu_data', Imu, self.calIMU)				#100 Hz
    rospy.spin()
    
  #%% Callback Encoders
  def callEncoder(self,data):
    # Measurement of sample time of encoders
    self.enc_toc= rospy.get_time()
    self.enc_Ts = (self.enc_toc - self.enc_tic)
    self.enc_tic = rospy.get_time()
    #%% Data descomposition
    string = data.data
    data_split = string.split(",")
    pulses_L = data_split[0].split(":")
    enc_pulses_L = float(pulses_L[1])
    pulses_R = data_split[1].split(":")
    enc_pulses_R = float(pulses_R[1])
    #%% Quit first data
    if (self.flag_enco):
      self.enc_pulses_R1 = enc_pulses_R
      self.enc_pulses_L1 = enc_pulses_L
      self.flag_enco = False
    delta_pulses_L = enc_pulses_L-self.enc_pulses_L1
    delta_pulses_R = enc_pulses_R-self.enc_pulses_R1
    #%% Conversion RPM to rad/seg
    self.wr = (2.0*np.pi/self.PPR)*((delta_pulses_R)/self.enc_Ts)
    self.wl = (2.0*np.pi/self.PPR)*((delta_pulses_L)/self.enc_Ts)
    #%% Displacement of a sample
    self.enc_pulses_R1 = enc_pulses_R
    self.enc_pulses_L1 = enc_pulses_L
    self.wr_2 = self.wr_1
    self.wl_2 = self.wl_1
    self.wr_1 = self.wr
    self.wl_1 = self.wl
    string = str(self.wr)+"\t"+str(self.wl)
    #%% Filter signals
    self.Filter_encoders()
    string = string+"\t"+str(self.wr)+"\t"+str(self.wl)+"\n"
    #file1 = open("data/EncoderFilter"+str(self.today)+".txt","a") 
    #file1.write(string)
    #file1.close() 
     #%% Kalman filter
    self.Kalman()
    
  #%% Filter
  def Filter_encoders(self):
    #%% Interpolation
    Dwr = (self.wr - self.wr_1)/self.enc_Ts
    Dwl = (self.wl - self.wl_1)/self.enc_Ts
    if (abs(Dwr) > self.acce):
      Delta_wr_1 = self.wr_1 - self.wr_2
      self.wr = self.wr_1 + Delta_wr_1
    if (abs(Dwl) > self.acce):
      Delta_wl_1 = self.wl_1 - self.wl_2
      self.wl = self.wl_1 + Delta_wl_1
    #%% Low-pass filter
    wr_f = 0.02*self.wr + 0.04*self.wr_1 + 0.02*self.wr_2 - (-1.56)*self.wr_f_1 - 0.64*self.wr_f_2
    wl_f = 0.02*self.wl + 0.04*self.wl_1 + 0.02*self.wl_2 - (-1.56)*self.wl_f_1 - 0.64*self.wl_f_2
    #%% Displacement of a sample
    self.wr_f_2 = self.wr_f_1
    self.wr_f_1 = wr_f
    self.wl_f_2 = self.wl_f_1
    self.wl_f_1 = wl_f
    # Output filtered variables
    self.wr = wr_f
    self.wl = wl_f
    
    
  #%% Callback IMU
  def calIMU(self,data):
    #%% Measurement of sample time of IMU
    self.imu_toc= rospy.get_time()
    self.imu_Ts = (self.imu_toc - self.imu_tic)
    self.imu_tic = rospy.get_time()
    #%% Data descomposition
    qx = data.orientation.x
    qy = data.orientation.y
    qz = data.orientation.z
    qw = data.orientation.w
    q = np.array([qx,qy,qz,qw])
    euler = tf.transformations.euler_from_quaternion(q)
    self.imu_pitch = euler[0]
    self.imu_roll  = euler[1]
#    self.imu_yaw   = euler[2]
    if (self.flag_imu):
      self.euler1 = euler
      print self.euler1
      self.flag_imu = False
    self.imu_yaw   = euler[2] - self.euler1[2]
#    print self.imu_yaw
#    if (self.imu_yaw < -3.0*np.pi/2.0):
#      self.imu_yaw = self.imu_yaw + 2.0*np.pi
#    elif (self.imu_yaw > 3.0*np.pi/2.0):
#      self.imu_yaw = self.imu_yaw - 2.0*np.pi
    if (self.imu_yaw < -np.pi):
      self.imu_yaw = self.imu_yaw + 2.0*np.pi
    elif (self.imu_yaw > np.pi):
      self.imu_yaw = self.imu_yaw - 2.0*np.pi
#    print self.imu_yaw
#    print "-------------------------------------"
    
  #%% Callback Kalman
  def Kalman(self):
    #%% Modelo skid-steer
    A = self.radio * np.matrix( [[1.0, 1.0] , [1.0/self.Yg, -1.0/self.Yg]] )
    U = np.matrix([[self.wr_f_1],[self.wl_f_1]])
    X = 0.5*np.dot(A,U)
    Delta_s = X[0,0]*self.enc_Ts
    Delta_theta = X[1,0]*self.enc_Ts
    # State space
    B = np.matrix([[Delta_s*np.cos(self.X_hat[2,0]+Delta_theta/2.0)],[Delta_s*np.sin(self.X_hat[2,0]+Delta_theta/2.0)],[Delta_theta]])
    self.X_hat = self.X_hat + B
    if (self.X_hat[2,0] < -3.0*np.pi/2.0):
      self.X_hat[2,0] = self.X_hat[2,0]+2.0*np.pi
    elif (self.X_hat[2,0] > 3.0*np.pi/2.0):
      self.X_hat[2,0] = self.X_hat[2,0]-2.0*np.pi
    # Jacobian computation
    a = -Delta_s*np.sin(self.X_hat[2]+Delta_theta/2.0)
    b =  Delta_s*np.cos(self.X_hat[2]+Delta_theta/2.0)
    Ak = np.matrix([[1.0, 0.0, a],[0.0, 1.0, b],[0.0, 0.0, 1.0]])
    # Matrix C
    Ck = np.matrix([0.0,0.0,1.0])
    #%% Error covariance matrix
    self.kalman_P =  np.dot(np.dot(Ak, self.kalman_P), Ak.T) + self.kalman_Q
    #%% Kalman filter Gain
    R = self.kalman_R + np.dot( np.dot(Ck,self.kalman_P) ,Ck.T)
    inv_R = np.linalg.inv( R )
    kalman_K = np.dot(np.dot(self.kalman_P,Ck.T),inv_R)
    #%% update variables
    # State space
    self.X_hat = self.X_hat + np.dot(kalman_K,(self.imu_yaw - self.X_hat[2,0]))
    # Error covariance matrix
    self.kalman_P =  np.dot((np.identity(3) - np.dot(kalman_K,Ck)),self.kalman_P)
    #%% Publication
    x = self.X_hat[0,0]
    y = self.X_hat[1,0]
    theta = self.X_hat[2,0]
    #print self.kalman_P
    self.publish_odom(x, y, theta, Delta_s, Delta_theta)
    
  #%% Callback Publish
  def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
    quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
    current_time = rospy.Time.now()
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x = cur_x
    odom.pose.pose.position.y = cur_y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation = Quaternion(*quat)
    odom.pose.covariance[0] = 0.01
    odom.pose.covariance[7] = 0.01
    odom.pose.covariance[14] = 99999
    odom.pose.covariance[21] = 99999
    odom.pose.covariance[28] = 99999
    odom.pose.covariance[35] = 0.01
    odom.child_frame_id = 'base_link'
    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = 0
    odom.twist.twist.angular.z = vth
    odom.twist.covariance = odom.pose.covariance
    self.ekf_pub.publish(odom)
    
  #%% MAIN
  def main(self):
    rospy.get_time()
    
if __name__ == '__main__':
    rospy.init_node("ekf_node")
    cv = NodoPos()
