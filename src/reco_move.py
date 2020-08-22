# -*- coding: utf-8 -*-
"""
Created on Sat Nov  2 09:00:44 2019

@author: u15280502270
"""

# -*- coding: utf-8 -*-
"""
Author: Harold F MURCIA

This is a temporary script file.
"""
import matplotlib.pyplot as plt
#from sklearn import datasets
import numpy as np
import pandas as pd
import torch
import sys, os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.legend import Legend
from datetime import datetime

PI=torch.acos(torch.Tensor([-1]))

dtype = torch.float

class alphaLiDAR(object):
    def __init__(self,data):
        self.data=data
        self.data_size=data.shape[0]
        print("Data size: "+str(data.shape))
        self.N = 1080     #UTM total samples per scan
        self.XYZINrRSaU_1=torch.zeros([self.data_size*self.N,9],dtype = torch.float,device=cuda0)
        self.XYZINrRSaU_2=torch.zeros([self.data_size*self.N,9],dtype = torch.float,device=cuda0)
        self.XYZINrRSaU_3=torch.zeros([self.data_size*self.N,9],dtype = torch.float,device=cuda0)
        self.gen(self.data,1)
        self.gen(self.data,2)
        self.gen(self.data,3)
        self.saveLOG()

    def gen(self,data,echo):
        hokuyo_start=7
        Number_of_returns=data.iloc[:,hokuyo_start+self.N*6:hokuyo_start+self.N*7]
        Number_of_returns=Number_of_returns.fillna(-1)
        Number_of_returns=torch.Tensor(Number_of_returns.values).cuda()
        scan_angle=data.iloc[:,hokuyo_start+self.N*7:hokuyo_start+self.N*8]
        scan_angle=torch.FloatTensor(scan_angle.values).cuda()
        if echo==1:
            print("1th echo")
            Range       = data.iloc[:,hokuyo_start:hokuyo_start+self.N]
            Intensity_df= data.iloc[:,hokuyo_start+self.N*3:hokuyo_start+self.N*4]
            Intensity_df= Intensity_df.fillna(-1)
            Intensity   = torch.Tensor(Intensity_df.values)
            self.XYZINrRSaU_1[:,4] = 1*torch.ones([self.data_size*self.N],dtype = torch.float,device=cuda0)
        elif echo==2:
            print(".........2th echo")
            Range           = data.iloc[:,hokuyo_start+self.N:hokuyo_start+self.N*2]
            Intensity_df=data.iloc[:,hokuyo_start+self.N*4:hokuyo_start+self.N*5]
            Intensity_df=Intensity_df.fillna(-1)
            Intensity       = torch.Tensor(Intensity_df.values)
            self.XYZINrRSaU_2[:,4]= 2*torch.ones([self.data_size*self.N],dtype = torch.float,device=cuda0)
        elif echo==3:
            print("..................3th echo")
            Range       = data.iloc[:,hokuyo_start+self.N*2:hokuyo_start+self.N*3]
            Intensity_df= data.iloc[:,hokuyo_start+self.N*5:hokuyo_start+self.N*6]
            Intensity_df= Intensity_df.fillna(-1)
            Intensity   = torch.Tensor(Intensity_df.values)
            self.XYZINrRSaU_3[:,4] = 3*torch.ones([self.data_size*self.N],dtype = torch.float,device=cuda0)
        else:
            print("Error")
        Range=Range.replace(60,0)
        row =Range.shape[0]
        col =Range.shape[1]
        Range_torch=torch.Tensor(Range.values)
        samples=row*col
        Range_torch=Range_torch.reshape(samples,1).cuda()
        #
        ## Traslation Matrix
        Tx1=0
        Ty1=0.04980
        Tz1=0.11350
        Tx2=0.04000
        Ty2=0
        Tz2=0.48675
        Tx3=0
        Ty3=0
        Tz3=0.36475
        roll_error  = -0*PI/180.0
        pitch_error = -0*5.23*PI/180.0
        yaw_error   = +0.0*PI/180.0
        ## LiDAR data
        beta = (torch.linspace(-135,135,self.N)*-PI/180.0)
        ## EKF pose
        ekf_X = data.iloc[:,1]
        ekf_Y = data.iloc[:,2]
        ekf_Z = data.iloc[:,3]
        ## IMU data
        pitch = -torch.Tensor(data.iloc[:,4].values).cuda()
        roll  = -torch.Tensor(data.iloc[:,5].values).cuda()
        aux=torch.Tensor(data.iloc[0:20,6].values).cuda()
        yaw_ini = torch.mean(aux).cuda()
        yaw = torch.Tensor(data.iloc[:,6].values)-yaw_ini
        X_LiDAR_echo = (torch.Tensor(Range.values)*torch.cos(beta)).cuda()
        Y_LiDAR_echo = (torch.Tensor(Range.values)*torch.sin(beta)).cuda()
        # Rotation Matrix around X
        alpha =1.0*torch.Tensor(data.iloc[:,0].values).cuda()
        L = len(alpha)
        input_matrix = torch.zeros([4,self.N],dtype = torch.float,device=cuda0)
        XYZ = torch.zeros([1,3],dtype = torch.float,device=cuda0)
        append_matrix = torch.zeros([3,self.N],dtype = torch.float,device=cuda0)
        # Constant Matrix
        T_LiDAR_axis     = torch.Tensor([[1,0,0,Tx1],[0,1,0,Ty1],[0,0,1,Tz1],[0,0,0,1]]).cuda()
        T_LiDAR_IMU      = torch.Tensor([[1,0,0,Tx2],[0,1,0,Ty2],[0,0,1,Tz2],[0,0,0,1]]).cuda()
        T_IMU_Origen     = torch.Tensor([[1,0,0,Tx3],[0,1,0,Ty3],[0,0,1,Tz3],[0,0,0,1]]).cuda()
        pitch_correction = torch.Tensor([[torch.cos(pitch_error), 0, torch.sin(pitch_error), 0], [0,1,0,0], [-torch.sin(pitch_error), 0, torch.cos(pitch_error), 0], [0,0,0,1]]).cuda()
        roll_correction  = torch.Tensor([[1,0,0,0],[0,torch.cos(roll_error),-torch.sin(roll_error), 0], [0,torch.sin(roll_error),torch.cos(roll_error), 0], [0,0,0,1]]).cuda()
        yaw_correction   = torch.Tensor([[torch.cos(yaw_error),-torch.sin(yaw_error),0,0], [torch.sin(yaw_error),torch.cos(yaw_error),0,0], [0, 0,1,0], [0,0,0,1]]).cuda()
        ##
        for k in range(0,L):
            # Sensor data [input]
            input_matrix[0,:] = X_LiDAR_echo[k,:]
            input_matrix[1,:] = Y_LiDAR_echo[k,:]
            # input_matrix in Z = zeros
            input_matrix[3,:] = torch.ones([1,self.N],device=cuda0)
            T_rot_y_dynamixel = torch.Tensor([[torch.cos(alpha[k]), 0, torch.sin(alpha[k]), 0], [0,1,0,0], [-torch.sin(alpha[k]), 0, torch.cos(alpha[k]), 0], [0,0,0,1]]).cuda()
            T_rot_pitch       = torch.Tensor([[torch.cos(pitch[k]), 0, torch.sin(pitch[k]), 0], [0,1,0,0], [-torch.sin(pitch[k]), 0, torch.cos(pitch[k]), 0], [0,0,0,1]]).cuda()
            T_rot_roll        = torch.Tensor([[1,0,0,0],[0,torch.cos(roll[k]),-torch.sin(roll[k]), 0], [0,torch.sin(roll[k]),torch.cos(roll[k]), 0], [0,0,0,1]]).cuda()
            T_rot_yaw         = torch.Tensor([[torch.cos(yaw[k]),-torch.sin(yaw[k]),0,0], [torch.sin(yaw[k]),torch.cos(yaw[k]),0,0], [0,0,1,0], [0,0,0,1]]).cuda()
            T_ekf             = torch.Tensor([[1,0,0,ekf_X[k]], [0,1,0,ekf_Y[k]],[0,0,1,ekf_Z[k]],[0,0,0,1]]).cuda()
            ##
            ## Transformations
            # Hokuyo deviation corrections
            P_axis_echo = torch.matmul(pitch_correction, input_matrix).cuda()
            P_axis_echo = torch.matmul(roll_correction, P_axis_echo).cuda()
            P_axis_echo = torch.matmul(yaw_correction, P_axis_echo).cuda()
            # Alpha Model
            P_axis_echo = torch.matmul(T_LiDAR_axis, P_axis_echo).cuda()
            P_axis_echo = torch.matmul(T_rot_y_dynamixel, P_axis_echo).cuda()
            P_IMU_echo  = torch.matmul(T_LiDAR_IMU, P_axis_echo).cuda()
            P_pitch     = torch.matmul(T_rot_pitch, P_IMU_echo).cuda()
            P_roll      = torch.matmul(T_rot_roll, P_pitch).cuda()
            P_yaw       = torch.matmul(T_rot_yaw, P_roll).cuda()
            P_ekf       = torch.matmul(T_ekf, P_yaw).cuda()
            P_Origen    = torch.matmul(T_IMU_Origen, P_ekf).cuda()
            #
            append_matrix[0,:]=P_Origen[0,:].cuda()
            append_matrix[1,:]=P_Origen[1,:].cuda()
            append_matrix[2,:]=P_Origen[2,:].cuda()
            XYZ = torch.cat((XYZ, append_matrix.T), dim=0).cuda()  ## from np.vstack (TEST)
        X=XYZ[1:,0].cuda()
        Y=XYZ[1:,1].cuda()
        Z=XYZ[1:,2].cuda()
        row =Intensity.shape[0]
        col =Intensity.shape[1]
        I=torch.zeros([row*col,1],dtype = torch.float,device=cuda0)
        Nr=torch.zeros([row*col,1],dtype = torch.float,device=cuda0)
        Sa=torch.zeros([row*col,1]).cuda()
        Rango=torch.zeros([row*col,1],dtype = torch.float,device=cuda0)
        Echo_position=torch.zeros([row*col,1],dtype = torch.float,device=cuda0)
        sample=0
        Range_array=torch.Tensor(Range.values).cuda()
        samples=row*col
        I=Intensity.reshape(samples,1).cuda()
        Sa=scan_angle.reshape(samples,1).cuda()
        Rango=Range_array.reshape(samples,1).cuda()
        Nr=Number_of_returns.reshape(samples,1).cuda()
        print("samples: "+str(samples))
        if echo==1:
            Echo_position=Echo_position+1
        elif echo==2:
            pos_2=torch.where(Nr==3)
            Echo_position[pos_2[0],0]=2
            pos_3=torch.where(Nr==2)
            Echo_position[pos_3[0],0]=3
        elif echo==3:
            Echo_position=Echo_position+3
        print("Max intensity: "+str(torch.max(I)))
        if echo==1:
            self.XYZINrRSaU_1[:,0]=X
            self.XYZINrRSaU_1[:,1]=Y
            self.XYZINrRSaU_1[:,2]=Z
            self.XYZINrRSaU_1[:,3]=I[:,0]
            self.XYZINrRSaU_1[:,5]=Nr[:,0]
            self.XYZINrRSaU_1[:,6]=Sa[:,0]
            self.XYZINrRSaU_1[:,7]=Echo_position[:,0]
            self.XYZINrRSaU_1[:,8]=Rango[:,0]
        elif echo==2:
            self.XYZINrRSaU_2[:,0]=X
            self.XYZINrRSaU_2[:,1]=Y
            self.XYZINrRSaU_2[:,2]=Z
            self.XYZINrRSaU_2[:,3]=I[:,0]
            self.XYZINrRSaU_2[:,5]=Nr[:,0]
            self.XYZINrRSaU_2[:,6]=Sa[:,0]
            self.XYZINrRSaU_2[:,7]=Echo_position[:,0]
            self.XYZINrRSaU_2[:,8]=Rango[:,0]
        elif echo==3:
            self.XYZINrRSaU_3[:,0]=X
            self.XYZINrRSaU_3[:,1]=Y
            self.XYZINrRSaU_3[:,2]=Z
            self.XYZINrRSaU_3[:,3]=I[:,0]
            self.XYZINrRSaU_3[:,5]=Nr[:,0]
            self.XYZINrRSaU_3[:,6]=Sa[:,0]
            self.XYZINrRSaU_3[:,7]=Echo_position[:,0]
            self.XYZINrRSaU_3[:,8]=Rango[:,0]


    def saveLOG(self):
        print("Generating Log file")
        XYZI = torch.cat((self.XYZINrRSaU_1, self.XYZINrRSaU_2),dim=0) # from  np.vstack
        XYZI = torch.cat((XYZI, self.XYZINrRSaU_3),dim=0).cpu() #  np.vstack
        pos=torch.where(XYZI[:,8]>0)
        XYZI=XYZI[pos[0],:]
        print("Max itensity:"+str(torch.max(XYZI[:,3]) ) )
        print("Min itensity:"+str(torch.min(XYZI[:,3]) ) )
        print("XYZI before")
        print(XYZI.shape)
        # Df ordena alfabeticamente
        df   = pd.DataFrame({'X':XYZI[:,0], 'Y':XYZI[:,1], 'Z':XYZI[:,2], 'I':XYZI[:,3], 'R':XYZI[:,4], 'Nr':XYZI[:,5], 'Sa':XYZI[:,6], 'Echoe_position':XYZI[:,7], 'Range':XYZI[:,8]})
        #print(df)
        del XYZI
        #print(df)
        drop_rows = df.loc[(df['X']==0) & (df['Y']==0) & (df['Z']==0)] # df.loc[(df['I']<=0) ]
        # Ecos llegan con intensidad nan o =0
        df   = df.drop(drop_rows.index)
        #XYZI = df.values
        Ruta=os.path.abspath('..')
        print("Path: "+str(Ruta))
        txt_fileNAME=os.path.join(Ruta,'data/reco_move_output.txt')
        df.to_csv(txt_fileNAME,index=None,sep='\t',mode='a')


if __name__ == "__main__":
    print("----------------------------------------------------------------------")
    print("Universidad de Ibagué -- Semillero de Investigación en Robótica SIRUI")
    print("txt2LAS alphaROVER")
    print("python3 reco_move.py fileinput[your_path] maximus_range[meters]")
    print("----------------------------------------------------------------------")
    fileName = sys.argv[1]
    dev = sys.argv[2]
    if dev == "cpu" or dev == "CPU":
        device = torch.device("cpu")
    else:
        torch.cuda.device_count()
        torch.cuda.get_device_name(0)
        torch.cuda.is_available()
        torch.cuda.init()
        cuda0 = torch.device("cuda:0")
    data1 = pd.read_csv(fileName, sep='\t')
    pCloud= alphaLiDAR(data1)
